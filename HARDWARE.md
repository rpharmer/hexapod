# Hexapod Hardware

<img src="illustrations/yellow2.png" height="300" />

This document contains information on the physical Hexapod; physical dimensions, hardware, components, controller board, pinouts, etc.

## Hexapod Mechanical Parts

The hexapod is 3D printed and I've used the Chipo build from [MakeYourPet](https://github.com/MakeYourPet/hexapod/tree/main). It's a fantastic repo containing all the needed 3D parts, parts list for screws, wiring, components, etcs. Easy to follow build instructions including videos and illustrations, wiring diagrams, firmware and an Android app plus some source code to get started.

<p float="left">
  <img src="illustrations/front-view.png" height="200" />
  <img src="illustrations/back-view.png" height="200" />
  <img src="illustrations/leg-components.png" height="200" />
  <img src="illustrations/tibia-components.png" height="200" />
</p>

## Electrical Components

I've used a 7.4 (2S) lipo battery. Due to the wide voltage range it is nessaccary to use a ubec to deliver consistent voltage to the servos. I've used MG996R servos.

### 7.4V (2S) Lipo Battery

The voltage range: is approx 8.4v fully charged and mustn't be discharged below 6v.

- 7.4V Nominal voltage
- 8.4V Fully charged
- 6.0V Minimum voltage (do not discharge below this as it can damage the battery)

I've used a 8A UBEC to regulate the voltage from the battery to the servos.

Rated:
- 8A continous
- 16A instantaneous

Input voltage
- 6V Minimum
- 36V Maximum

Output voltage (selectable using jumper)
- 5.2V
- 6.0V
- 7.4V
- 8.4V

### MG996R Servo Specs

Stall Torque:
- 13 kg / cm (4.8V)
- 15 kg / cm (6.0V.

Operating Speed:
- 0.17 sec / 60 degrees (4.8V no load)
- 0.14 sec / 60 degrees (6.0V no load)

Operating voltage: 4.8V - 7.2V.
Dimension: 40mm x 19mm x 43mm.

## Wiring Diagram
<img src="https://github.com/MakeYourPet/hexapod/blob/7d8fc8034d715d1c9373f48281ecfa500c994d8b/wiring-diagram-servo2040.png" height="400">

Place UBEC inline between battery and relay.

## Servo 2040

<img src ="https://www.kiwi-electronics.com/image/cache/catalog/product/83habfak/servo-2040-2-1600x1066h.jpg" height="300">

This is the powerful servo controller, able to drive 18 servos, read current draw and voltage of the servo power rail, 6 analog sensors used for limit switches on the end of each foot, 6 addressable RGB LEDs suitable for visual feedback, 3 ADC input/output pins (one is used to drive the relay to enable/disable the battery), Serial Wire Debug pins which are used to upload the firmware using OpenOCD. The heart of the servo 2040 board is a Raspberry Pi RP2040 (pico) (Dual Arm Cortex M0+ running at up to 133Mhz with 264kB of SRAM) also has 2MB of QSPI flash supporting XiP.
A full schematic can be found [here](https://cdn.shopify.com/s/files/1/0174/1800/files/servo2040_schematic.pdf)
A pdf development guide for the pico can be found [here,](https://cdn.shopify.com/s/files/1/0174/1800/files/servo2040_schematic.pdf) this contains important instructions for flashing the board using OpenOCD.
I'm using a Raspberry Pi 5 as main board of the hexapod, you will need to use the config file provided [here](https://forums.raspberrypi.com/viewtopic.php?t=362826) for flashing the servo 2040 from a rpi5.

Our firmware is written in c++ under the directory hexapod-client. Pimoroni provides guides to setting up the enviroment for pico development (which applies to the servo 2040 board) and a boiler plate project. Links to these can be found here; [Boilerplate project](https://github.com/pimoroni/pico-boilerplate), [Pico SDK setup instructions](https://github.com/pimoroni/pimoroni-pico/blob/main/setting-up-the-pico-sdk.md), [Servo 2040 example code](https://github.com/pimoroni/pimoroni-pico/tree/main/examples/servo2040) and [Pico example code](https://github.com/raspberrypi/pico-examples?tab=readme-ov-file).

## Dimensions And Angles

Length of leg segments in millimeters
COXA_LEN 43
FEMUR_LEN 60
TIBIA_LEN 104

Distance between the coxa rotation centers of different legs in millimeters.
L1_TO_R1 126
L1_TO_L3 167
L2_TO_R2 163

The height where the legs connect to the frame.
LEG_CONNECTION_Z -7

The Z value for the leg endpoints when sitting on a flat surface.
LEG_SITTING_Z -40

The angle between the servo itself and the leg segment when the servo is centered.
COXA_ATTACH_ANGLE -8
FEMUR_ATTACH_ANGLE 35
TIBIA_ATTACH_ANGLE 83





