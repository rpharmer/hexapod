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

I've used a 7.4 (2S) lipo battery. The voltage range is approx 8.4v fully charged and mustn't be discharged below 6v. Due to the wide voltage range it is nessaccary to use a 8A ubec to deliver consistent voltage to the servos. I've used MG996R servos.

## MG996R Servo Specs

Stall Torque:
- 13 kg / cm (4.8V)
- 15 kg / cm (6.0V.

Operating Speed:
- 0.17 sec / 60 degrees (4.8V no load)
- 0.14 sec / 60 degrees (6.0V no load)

Operating voltage: 4.8 - 7.2V.
Dimension: 40mm x 19mm x 43mm.

