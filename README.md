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

Currently the firmware does not program the battery gauge of Blinkekatzen by itself.  
An experimental method for programming battery gauges is implemented and can be enabled
through menuconfig. If you enable this option please make sure the ESP32 is not reset
during the programming operation. Programming of the gauge is only done once during
initial bootup and may take up to a minute. If the ESP is reset during the programming
operation the battery gauge can end up in a bricked state where it totally jams
communication on the I2C bus, pulling SCL and/or SDA low continously.

As an alternative BQStudio and an EV2400 or similar interface can be used for programming
via I2C connector J3.

## Supported batteries

Currently Blinkekatzen come with only one battery gauge profile. This profile has been
built for Samsung INR18650-35E LiIon cells. If another type is to be used with Blinkekatzen
it is recommended to add a profile for that specific battery type through TI BQStudio. If
this step is omitted runtime and remaining capacity estimations will be significantly off.

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

Within the python venv provided for esp-idf a version of pypng and numpy for python3 must be available.
The dependencies can be directly installed through pip (`pip3 install -r requirements.txt`).

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

## Flashing

Flashing this firmware onto Blinkekatzen is very easy thanks to the USB Serial and JTAG peripheral integrated into the ESP32-C3. Just connecting to the USB-C port of
a Blinkekatze will make a ACM USB serial device available that can be used for flashing via esp-idf. On a Linux system `idf.py -p /dev/ttyACM0 build flash monitor`
will commonly be the correct command to build and flash the firmware and enter serial monitor mode afterwards.

### Troubleshooting

Depending on the exact vendor and manufacturer some ESP32-C3 MINI modules can come preflashed with firmware that reconfigures the USB pins to a different function.
For those modules it might be ncessary to use the strapping pins exposed via header J2 on the Blinkekatze. If any problems are encoutered during flashing it might help
to short IO9 on J2 to GND and - with IO9 still connected to GND - connect EN to GND and disconect it again. The connection between IO9 and GND can now be removed.
This puts the ESP32-C3 into USB boot mode. From here on flashing as normal should be possible.
