Blinkekatze firmware
====================

This repository contains the firmware for the Blinkekatze project.

# Features

 - Wireless synchronization of large Blinkekatzen groups
 - OTA-first software updates
 - Interactive light display across multiple Blinkekatze
 - Full color calibration for accurate color representation
 - USB shell for management of Blinkekatzen groups

# Limitations

Currently the firmware can not program the battery gauge of Blinkekatzen by itself.  
There is some infrastructure to implement this in the future however it will likely
require adding a soft I2C implementation due to extreme response times and clock
stretching of the gauge beyond what the ESP32-C3 supports in hardware.  
I'm using BQStudio and an EV2400 for programming at the moment.

# Dependencies

The Blinkekatze firmware is based on the esp-idf development framework.
In addition to esp-idf opencv-python is also required to process color calibration tables.

## Installing esp-idf

The appropriate esp-idf framework version is embedded in this repository as a submodule.
To install esp-idf for basic use first make sure submodules are fully cloned by running  
`git submodule update --init --recursive`  
from the root of this repository.  
[The comprehensive installation guide provided by Espressif](https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32c3/get-started/index.html#installation)
details further steps required for installation.

## Additional dependencies

Within the python venv provided for esp-idf a version of opencv-python for python3 (cv2 API) must be available.
It can be directly installed through pip (`pip3 install opencv-python`).

# Usage

This repository contains a standard esp-idf project, thus all usual building, installing and debugging steps outlined
in [the esp-idf documentation](https://docs.espressif.com/projects/esp-idf/en/release-v5.1/esp32c3/get-started/index.html#build-your-first-project) apply.

## Configuration

The Blinkekatze firmware provides a set of compile time and runtime configuration options.

### Compile time

The `Component config -> Blinkekatze` subtree of esp-idf Kconfig provides a number of configuration options relating to the wireless interface used for communication
between Blinkekatzen.

### Runtime

At runtime a set of configuration options is available through a serial shell on the USB port of the Blinkekatzen. Autocompletion and a `help` command are available.
