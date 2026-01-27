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
    PlotInfo("Ascent Velocity", "m/s", "velocity", 2, 2)
]

datapoints: list[DataPoint] = []

def create_graph(plot: PlotInfo, x_data, y_data) -> dbc.Col:
    layout = dict(
        title = dict(text = plot.fancy_name),
        yaxis = dict(autoscale=True, title=dict(text = plot.label))
    )

    if x_data:
        # If provided x_data, create a grap hwith the last 60 seconds of data
        max_x = x_data[-1]
        layout["xaxis"] = dict(range = [max_x-60, max_x])

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
    dcc.Checklist(id="options", options=[dict(label="Autoscale 60", value="autoscale")]),

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
def autoscale_changed(_):
    # When autoscale changes, we need to reset the graphs
    children = [create_graph(plot_info[i], None, None) for i in range(5)]
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
                # Remember, time is in ms.
                # We do this after adding the point so that we have 1 point off the screen,
                # which makes the scrolling effect look correct
                if latest_time - packet.time > 60:
                    break

            x_axis.reverse()
            y_axis.reverse()

            children.append(create_graph(plot_info[i], x_axis, y_axis))

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

        return no_update, len(datapoints)


"""
@callback(
   Output("processed-packets", "data"),
   Input("interval", "n_intervals"),
   State("options", "value"),
   State("processed-packets", "data"),
)
def update_plots(_, options, processed_packets):
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

    return len(datapoints)
"""

# --------------------[ Main Program ]--------------------

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
            datapoints.append(TelemetryPacket(
                i,
                i,
                0,
                i * 10,
                40*np.sin(i),
                5 + random.random() * 10,
                0
            ))
            i += 1

            time.sleep(1)

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
                print("Packet:", packet)
                if packet.header.packet_type == PacketType.TELEMETRY:
                    datapoints.append(
                        telemetry_packet_to_datapoint(packet.data)
                    )
                print()
            except:
                continue

    elif args.from_file:
        # Read in all the datapoints and graph them
        lines = open(args.from_file, 'r').readlines()
        for line in lines:
            data = [float(n) for n in line.strip().split(',')]
            packet = DataPoint(*data)
            datapoints.append(packet)

        
t = threading.Thread(target=run)
t.start()

app.run()
# On CTRL-C, app.run() terminates, so lets clean up
running = False
t.join(1)