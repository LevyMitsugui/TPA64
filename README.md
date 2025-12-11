# TPA64 – Python Driver for the USB-TPA64 Thermal Camera

A lightweight Python interface for the USB-TPA64 8×8 thermal camera module.
Provides access to ambient temperature, pixel data, full frames, and optional image generation.

Protocol details implemented in this library follow the official device documentation.

---

## Installation
### Requirements

This library depends on:
```bash
pyserial
pillow
```

Install them manually if needed:
```bash
pip install pyserial pillow
```
### Install the package (development mode)

Inside the project root:

pip install -e .

This installs the ```tpa64``` package and allows modifying the source without reinstalling.

---

## Linux Setup

The camera usually appears as:

- ```/dev/ttyUSB0```

- ```/dev/ttyACM0```

If you get **permission denied**, add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
```

Then log out and back in.

Or temporarily:
```bash
sudo chmod 666 /dev/ttyUSB0
```

---
## Running the Examples

Two examples are included in src/Examples/:

### 1. Streaming temperatures

```streamNoView.py```
Shows live ambient, min, and max temperatures.


Run:
```bash
python3 -m Examples.streamNoView
```

or from inside src/:
```bash
python3 Examples/streamNoView.py
```

---
### 2. Capturing a frame and saving an image

```pngCapture.py```
Captures one frame and saves snap.png.


Run:
```bash
python3 -m Examples.pngCapture
```

or:
```bash
python3 Examples/pngCapture.py
```

---
## Basic Usage (Overview)
```python
from tpa64 import USBTPA64

with USBTPA64("COM3") as cam:
    frame = cam.get_frame()
    print(frame.ambient_c, frame.min_c, frame.max_c)
```

The full API includes:

- ```get_info()``` – module ID & firmware

- ```get_serial()``` – serial number

- ```get_ambient()``` – thermistor

- ```get_pixels()``` – 8×8 array

- ```get_frame()``` – ambient + pixels

- ```iter_frames()``` – continuous streaming
