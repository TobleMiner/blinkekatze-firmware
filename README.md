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

Currently the firmware does not program the battery gauge of Blinkekatzen by default.  
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
In addition to esp-idf pypng is also required to process color calibration tables.

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

# Color palette IDs

| ID | Palette |
|---|---|
| 0 | CloudColors_p |
| 1 | LavaColors_p |
| 2 | OceanColors_p |
| 3 | ForestColors_p |
| 4 | wled_ib_jul01_gp |
| 5 | wled_es_vintage_57_gp |
| 6 | wled_es_vintage_01_gp |
| 7 | wled_es_rivendell_15_gp |
| 8 | wled_rgi_15_gp |
| 9 | wled_retro2_16_gp |
| 10 | wled_Analogous_1_gp |
| 11 | wled_es_pinksplash_08_gp |
| 12 | wled_es_ocean_breeze_036_gp |
| 13 | wled_departure_gp |
| 14 | wled_es_landscape_64_gp |
| 15 | wled_es_landscape_33_gp |
| 16 | wled_rainbowsherbet_gp |
| 17 | wled_gr65_hult_gp |
| 18 | wled_gr64_hult_gp |
| 19 | wled_GMT_drywet_gp |
| 20 | wled_ib15_gp |
| 21 | wled_Tertiary_01_gp |
| 22 | wled_lava_gp |
| 23 | wled_fierce_ice_gp |
| 24 | wled_Colorfull_gp |
| 25 | wled_Pink_Purple_gp |
| 26 | wled_Sunset_Real_gp |
| 27 | wled_Sunset_Yellow_gp |
| 28 | wled_Beech_gp |
| 29 | wled_Another_Sunset_gp |
| 30 | wled_es_autumn_19_gp |
| 31 | wled_BlacK_Blue_Magenta_White_gp |
| 32 | wled_BlacK_Magenta_Red_gp |
| 33 | wled_BlacK_Red_Magenta_Yellow_gp |
| 34 | wled_Blue_Cyan_Yellow_gp |
| 35 | wled_Orange_Teal_gp |
| 36 | wled_Tiamat_gp |
| 37 | wled_April_Night_gp |
| 38 | wled_Orangery_gp |
| 39 | wled_C9_gp |
| 40 | wled_Sakura_gp |
| 41 | wled_Aurora_gp |
| 42 | wled_Atlantica_gp |
| 43 | wled_C9_2_gp |
| 44 | wled_C9_new_gp |
| 45 | wled_temperature_gp |
| 46 | wled_retro_clown_gp |
| 47 | wled_candy_gp |
| 48 | wled_toxy_reaf_gp |
| 49 | wled_fairy_reaf_gp |
| 50 | wled_semi_blue_gp |
| 51 | wled_pink_candy_gp |
| 52 | wled_red_reaf_gp |
| 53 | wled_aqua_flash_gp |
| 54 | wled_yelblu_hot_gp |
| 55 | wled_lite_light_gp |
| 56 | wled_red_flash_gp |
| 57 | wled_blink_red_gp |
| 58 | wled_red_shift_gp |
| 59 | wled_red_tide_gp |
| 60 | wled_candy2_gp |
| 61 | wled_trafficlight_gp |
| 62 | wled_Aurora2_gp |
| 63 | PartyColors_gc22 |
| 64 | RainbowColors_gc22 |
| 65 | RainbowStripeColors_gc22 |
