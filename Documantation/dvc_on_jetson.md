# DVC running on Nvidia Jetson Xavier NX

## R/W data on SD card

To get image of your card on linux system just go to terminal and write:
```
sudo dd if=/dev/{NameOfDisc} of=/{CustomName}.img
```
To get back your content use:
```
sudo dd if=/{CustomName}.img of=/dev/{NameOfDisc}
```
Name of your device should be visible in discs (look picture below)

![PIC1](discs.jpg)

## Powering Jetson

Jetson runs in range from 9V to 19V and gets maximum current of 5A while on 9V.

## Setup Jetson 

1. Download Etcher: https://www.balena.io/etcher#download-etcher
2. Upload your chosen image 
3. Boot and configure

## Setting up Ardupilot on Pixhawk

Open qGroundStation 
```
./QGroundControl.AppImage
```
Click on Q -> Vehicle Setup -> Firmware 

Once you are in this window go ahead and replag the px.

Default configuration shall be fine :) 

