import struct
import enum
from dataclasses import dataclass
import typing
import serial
import argparse
import numpy as np
import time
import threading
from dash import Dash, Output, html, dcc, Input, State, callback, set_props, no_update
import dash_bootstrap_components as dbc
import random
import logging
import math
import random

# --------------------[ Packets ]--------------------

class PacketType(enum.Enum):
    PING = 1
    TELEMETRY = 2

@dataclass
class PingPacket:
    counter: int

# Ideally this would be a 1 to 1 representation
# of the C code, but to make things simpler the
# fixed-point numbers are made floating-point during deserialization.
# Thus, this essentially represents a struct of fully deseriaized telemetry data
@dataclass
class TelemetryPacket:
    frame_count: int
    time: int
    altitude: int
    pressure: int
    temperature: int
    acceleration_magnitude: int
    velocity: int

# Contains general information about the packet
@dataclass
class PacketHeader:
    satellite_id: int
    packet_type: PacketType

# The full packet that is sent/recieved
@dataclass
class Packet:
    header: PacketHeader
    data: TelemetryPacket | PingPacket

# A fully deserialized telemetry packet, with all fixed-point
# numbers conveted to floating-point
@dataclass
class DataPoint:
    frame_count: int
    time: float
    altitude: float
    pressure: float
    temperature: float
    acceleration_magnitude: float
    velocity: float

def deserialize_packet(raw: bytes) -> typing.Optional[Packet]:
    # see https://docs.python.org/3/library/struct.html
    # esp32 is little endian
    # <2H = little endian, 2 16bit uints
    id, ty = struct.unpack("<2H", raw[:4])
    try:
        ty = PacketType(ty)
    except ValueError:
        print(f"Unknown packet: {ty}")
        return None
    
    header = PacketHeader(id, ty)
    data = None
    match header.packet_type:
        case PacketType.PING:
            # <I = little endian, 1 32bit uint
            data = PingPacket(struct.unpack("<I", raw[4:]))
        case PacketType.TELEMETRY:
            # <6Ii = little endian, 2 32bit uints, 1 32bit int, 3 32bit uints, 1 32bit int, 
            raw = list(struct.unpack("<2Ii3Ii", raw[4:]))
            data = TelemetryPacket(*raw)
    
    return Packet(header, data)

def serialize_packet(packet: Packet) -> bytes:
    # <2B = little endian, 2 bytes
    b = struct.pack("<2B", packet.header.satellite_id, packet.header.packet_type)

    match packet.header.packet_type:
        case PacketType.PING:
            b += struct.pack("<I", packet.data.counter)
        # We never send telemetry packets so we dont need the code for it

    return b

def telemetry_packet_to_datapoint(packet: TelemetryPacket) -> DataPoint:
    return DataPoint(
        packet.frame_count,
        # These are all fixed-point floats with 3 decimals
        packet.time / 1000,
        packet.altitude / 1000,
        packet.pressure / 1000,
        packet.temperature / 1000,
        packet.acceleration_magnitude / 1000,
        packet.velocity / 1000
    )

# --------------------[ Plotting ]--------------------

# Contains the information we need to create a graph.
# Helps us create a lot of graphs easily.
@dataclass
class PlotInfo:
    fancy_name: str
    label: str
    data_name: str
    row: int
    col: int

plot_info = [
    PlotInfo("Temperature", "°C", "temperature", 1, 1),
    PlotInfo("Pressure", "pa", "pressure", 1, 2),
    PlotInfo("Altitude", "m", "altitude", 1, 3),
    PlotInfo("Acceleration Magnitude", "g", "acceleration_magnitude", 2, 1),
    PlotInfo("Ascent Velocity", "m/s", "velocity", 2, 2),
]
packet_loss_plot_info = PlotInfo("Packet Loss", "%", "packet_loss", 2, 3)

# Calculates packet loss % over this amount of packets
# 20 = ~10s if there's 2 packets per second
PACKET_LOSS_RANGE = 20

datapoints: list[DataPoint] = []
received_packets: list[bool] = []
packet_loss_graph: list[list[int]] = [[], []]

def create_graph(plot: PlotInfo, x_data, y_data) -> dbc.Col:
    layout = dict(
        title = dict(text = plot.fancy_name),
        yaxis = dict(autoscale=True, title=dict(text = plot.label))
    )

    layout["xaxis"] = dict(
        showline = True,
        mirror = True,
        zerolinecolor = "#AAA",
    )
    layout["yaxis"] = layout["xaxis"].copy()
    if x_data:
        # If provided x_data, create a graph with the last 60 seconds of data
        max_x = x_data[-1]
        layout["xaxis"]["range"] = [max_x-60, max_x]
        
    if plot.data_name == "packet_loss":
        layout["yaxis"]["range"] = [0, 1]

    return dbc.Col(dcc.Graph(id=plot.data_name, figure=dict(
        # data is a list that contains multiple lines, but we only have 1 line per graph
        data = [dict(x = x_data or [], y=y_data or [])],
        layout = layout,
    )), lg=4, md=12) # There are 12 columns for the total layout, so 4 == 1/3 width and 12 == full width

# Dash uses flask internally
# Suppress dash output to terminal except for errors
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Dash("KSC TSAT-2B Live Telemetry Monitor", external_stylesheets=[dbc.themes.BOOTSTRAP])
ksc_image = dbc.Col(html.Img(src=app.get_asset_url("ksc.png")), lg=4, md=12, style=dict(textAlign="center"))

app.layout = [
    dcc.Interval(id="interval", interval=250), # The interval to update the graphs when we recieve packets
    dcc.Store(id="processed-packets", data=0), # The processed data points for each graph so far.
    dcc.Checklist(id="options", options=[
        dict(label="Autoscroll 60s", value="autoscale"),
        dict(label="Show Packet Loss", value="packetloss"),
    ]),

    dbc.Container([
        dbc.Row(id="container")
    ], fluid=True),
]

@callback(
    Output("container", "children", allow_duplicate=True),
    Output("processed-packets", "data", allow_duplicate=True),
    Input("options", "value"),
    prevent_initial_call = 'initial_duplicate'
)
def options_changed(options):
    # When options change, we need to reset the graphs
    children = [create_graph(plot_info[i], None, None) for i in range(5)]
    if options and "packetloss" in options:
        children.append(create_graph(packet_loss_plot_info, None, None))
    else:
        children.append(ksc_image)

    return children, 0


@callback(
    Output("container", "children", allow_duplicate=True),
    Output("processed-packets", "data", allow_duplicate=True),
    Input("interval", "n_intervals"),
    State("options", "value"),
    State("processed-packets", "data"),
    prevent_initial_call = 'initial_duplicate'
)
def update(_, options, processed_packets):
    if options and "autoscale" in options:
        if len(datapoints) == 0:
            return no_update, no_update
        latest_time = datapoints[-1].time
        # Create graph data
        children = []
        for i,plot in enumerate(plot_info):
            x_axis = []
            y_axis = []
            # Go through each datapoint in reverse order
            for idx in range(len(datapoints)-1, 0, -1):
                packet = datapoints[idx]

                # Add the points
                x_axis.append(packet.time)
                y_axis.append(getattr(packet, plot.data_name))

                # If we are 60 seconds behind the latest datapoint, break
                # We do this after adding the point so that we have 1 point off the screen,
                # which makes the scrolling effect look correct
                if latest_time - packet.time > 60:
                    break

            x_axis.reverse()
            y_axis.reverse()

            children.append(create_graph(plot_info[i], x_axis, y_axis))

        if options and "packetloss" in options:
            x_axis = []
            y_axis = []
            for idx in range(len(packet_loss_graph[0])-1, 0, -1):
                data_time = packet_loss_graph[0][idx]
                data_value = packet_loss_graph[1][idx]

                x_axis.append(data_time)
                y_axis.append(data_value)

                # See above
                if packet_loss_graph[0][-1] - data_time > 60:
                    break

            x_axis.reverse()
            y_axis.reverse()
            
            children.append(create_graph(packet_loss_plot_info, x_axis, y_axis))
        else:
            children.append(ksc_image)

        # Reset the processed_packets counter so when we switch back from this autoscaling it correctly updates the graphs
        return children, no_update
    else:
        # Take in new telemetry data
        for plot in plot_info:
            # This is the additional graph data we append
            new_data_x, new_data_y = [], []
            # Look through all the packets we haven't processed yet
            for idx in range(processed_packets, len(datapoints)):
                packet = datapoints[idx]
                new_data_x.append(packet.time)
                new_data_y.append(getattr(packet, plot.data_name))

            props = dict(extendData=[dict(x=[new_data_x], y=[new_data_y])])

            # Setting the extendData property appends the data to the graph
            set_props(plot.data_name, props)

        if options and "packetloss" in options:
            new_data_x, new_data_y = [], []
            if len(datapoints) != 0 and processed_packets < len(datapoints):
                range_start = datapoints[processed_packets-1].frame_count+1 if processed_packets > 0 else 0
                for idx in range(range_start, datapoints[-1].frame_count+1):
                    new_data_x.append(packet_loss_graph[0][idx])
                    new_data_y.append(packet_loss_graph[1][idx])

                props = dict(extendData=[dict(x=[new_data_x], y=[new_data_y])])

                set_props(packet_loss_plot_info.data_name, props)

        return no_update, len(datapoints)


# --------------------[ Main Program ]--------------------

# Updates the packet loss graph.
# ONLY call this when we receive a packet.
# and call this EVERY time we receive a packet
def update_packet_loss():
    if len(datapoints) == 0:
        return
    
    # Mark all missed packets
    received_packets.extend(False for _ in range(len(received_packets), datapoints[-1].frame_count))
    # Add the received packet
    received_packets.append(True)

    # Update the graph.
    if len(datapoints) == 1:
        prev_framecount = -1
        prev_time = 0
    else:
        prev_framecount = datapoints[-2].frame_count
        prev_time = datapoints[-2].time

    range_begin = prev_framecount+1
    range_end = datapoints[-1].frame_count+1
    time_end = datapoints[-1].time
    for frame_count in range(range_begin, range_end):
        # If we miss say 4 packets, we interpolate the time between
        # the previous packet and current packet 4 times to create
        # points for each packet we missed.
        interpolated_time = (frame_count - range_begin) / (range_end - range_begin) * (time_end - prev_time) + prev_time
        loss_range_begin = max(0, frame_count + 1 - PACKET_LOSS_RANGE)
        packet_range_received = sum(received_packets[loss_range_begin:frame_count+1])
        packet_loss_percent = 1 - packet_range_received / (frame_count+1-loss_range_begin)
        packet_loss_graph[0].append(interpolated_time)
        packet_loss_graph[1].append(packet_loss_percent)

parser = argparse.ArgumentParser(
    description = "TSAT-2B Telemetry Monitor"
)
action_group = parser.add_mutually_exclusive_group(required=True)
action_group.add_argument("--live", metavar="port", help="Recieves live telemetry from a serial ground reciever. Port is usually COMX on Windows and /dev/ttyUSBX or /dev/ttySX on Linux/Mac where X is a number.")
action_group.add_argument("--from-file", metavar="file", help="Reads telemetry data from a file and displays it.")
action_group.add_argument("--test-graphs", action="store_true", help="Debugging utility. Creates random data for graphs to test the display.")
args = parser.parse_args()

running = True
def run():
    if args.test_graphs:
        # Create random test data to ensure the graphs work correctly
        i = 0

        while running:
            # Simulate some packet loss
            if random.randint(1, 5) != 1:
                datapoints.append(TelemetryPacket(
                    i,
                    i,
                    0,
                    i * 10,
                    40*np.sin(i/2),
                    5 + random.random() * 10,
                    0
                ))
                update_packet_loss()

            i += 1


            time.sleep(0.5)

    elif args.live:
        # Take in live data from a reciever and graph the telemetry packets
        ser = serial.Serial(args.live, baudrate=115200)

        while running:
            raw = ser.readline()
            try:
                raw = raw.decode().strip()
                print("Raw:", raw)
                line = bytes([int(i) for i in raw.split(' ')])
                packet = deserialize_packet(line)
                # If we already received a packet or received future packets, skip.
                if len(packet_loss_graph[0]) <= packet.data.frame_count:
                    print("Packet:", packet)
                    if packet.header.packet_type == PacketType.TELEMETRY:
                        datapoint = telemetry_packet_to_datapoint(packet.data)
                        datapoints.append(datapoint)
                        update_packet_loss()                    

                print()
            except:
                continue

    elif args.from_file:
        # Read in all the datapoints and graph them
        lines = open(args.from_file, 'r').readlines()
        # Skip first line (header line)
        for line in lines[1:]:
            data = [float(n) for n in line.strip().split(',')]
            datapoint = DataPoint(
                int(data[0]),
                data[1]/1000,
                data[4],
                data[3],
                data[2],
                math.sqrt(data[5]**2 + data[6] ** 2 + data[7] ** 2),
                0,
            )
            if len(datapoints) > 0:
                datapoint.velocity = (datapoint.altitude - datapoints[-1].altitude) / (datapoint.time - datapoints[-1].time)
            datapoints.append(datapoint)
            update_packet_loss()

        
t = threading.Thread(target=run)
t.start()

app.run()
# On CTRL-C, app.run() terminates, so lets clean up
running = False
t.join(1)