# Ground Visualization

This runs on a computer on the ground, connected to a ground station ESP via serial USB.

## Installation Requirements

Ensure python is installed. Create a virtual environment:
```sh
python -m venv venv
```

Activate your virtual environment:
```sh
# Windows
.\venv\Scripts\activate.bat

# Linux
source ./venv/bin/activate
```

Install requirements:
```sh
pip install -r requirements.txt
```

## Operation

Run the program:
```sh
# Live telemetry
# Port is usually COM0/COM1 on windows, and /dev/ttyUSB0 on linux.
python main.py --live /dev/ttyUSB0

# Visualizing data from file (recovered from SD card)
python main.py --from-file tsatlog1.csv
```

Then open up the webpage in your browser, typically `localhost:8050`

For live telemetry, follow the following steps:

1. Connect an RFM to an ESP via PCB or breadboard.
2. Flash the `ground_pipe` software to the ESP.
3. Check which port the ESP is connected to your computer.
4. Run the program with the `--live <PORT>` argument.