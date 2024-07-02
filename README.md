# Summary
This repo provides a python library to control LoRa chips that use SPI from a raspberry Pi 5.
The library supports both transmission and reception of LoRa frames.
We also provide a sample file to configure a custom chip enable pin, sometime necessary for some Raspberry Pi HATs. The given example is compatible with the Dragino LoRa/GPS HAT.

The configuration of the following parameters is supported:
- Spreading factor
- Coding rate
- Center frequency
- Bandwidth
- Sync words (Network ID)
- Payload content
- Explicit or Implicit header mode
- Cyclic redundancy check (CRC)
- Transmit power
- Low data rate optimization

# Installation and Usage
- Simply clone the repository
- Enable SPI on your raspberry Pi using ```sudo raspi-config``` and enabling SPI under _Interface Options_
- Modify the PINs connected to the _reset_, _DIO 0_ of your chip in the beginning of loralibPi5.py if needed
- Modify the SPI _BUS_ and _DEVICE_ you are using  with your Raspiberry Pi
    - You can see wich device and bus are enables on your pi using ```ls /dev/spidev*```. The results are _\dev\spidev{BUS}.{DEVICE}_
    - The default SPI BUS 0 uses GPIO 9, 10 and 11
    - Set the device based on the chip enable pin you want to use (0 for CE0 on GPIO 7, 1 for CE1 on GPIO 8) in the beginning of lo ralibPi5.
    - If you want to use a custom pin for chip enable, see the section below
- Sample receiver and transmitter codes are provided

## Custom Chip Enable Pin
- The file _spi-cs-extend.dts_ provides an overlay that can be used to create a new SPI Device with a custom chip enable pin (gpio 25 in the example).
- Compile the file to a binary using

    ```dtc -@ -I dts -O dtb -o spi-cs-extend.dtbo spi-cs-extend.dts```
- Then place `spi-cs-extend.dtbo` into `/boot/overlays`
- Add the following line to your `/boot/firmware/config.txt`: 
  
  ```dtoverlay=spi-cs-extend```
- Reboot
- You should find that a new device /dev/spidev0.2 has been created, where 0 corresponds to the _BUS_ and 2 to the _DEVICE_



