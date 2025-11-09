# HTPC-BLEBoot 

## Overview

ESP32 based Bluetooth Low Energy to USB HID bridge that allows remapping of BLE remote control keys to either any keyboard input or driving the ESP's gpio. 
When coupled with opto-isolators, this allows you to do things like trigger button presses on other equiptment.

I mainly made this project to use a cheap android & windows compatible BLE remote as a power switch for my Home Theater PC (hence the name).

However I made the project in such a way that it can be used for anything really.

All HID data that the remote normally transmits is passed through to the host machine via USB, such as keyboard(s), media keys and mouse data. With the option for reconfigured keys to be blocked from being seen by the host machine.

I have included a quick GUI for flashing the same Dev board I used, to help new users.


I have tested this project with a cheap "Air Mouse" remote from china, other BLE devices should also work...

See below for links to the exact model..

## Core Features

### BLE Remote Control
- **HID Emulation**: Acts as USB keyboard/mouse to control HTPC
- **Auto-Reconnect**: Automatically reconnects to last paired BLE remote
- **Battery Monitoring**: Displays connected remote battery percentage in web ui
- **Connection Status**: Visual feedback with rainbow LED animation on successful connection (can be disabled)

### Web Interface

#### Main Menu
- Device connection status with battery percentage
- System information, HID descriptor map from remote
- View/clear paired remote 


#### Mapping Menu
- Map up to 64 buttons on a remote to either, Trigger a GPIO action, Trigger LED action or Remap to a keyboard key
- Time to live; if a button is mapped to GPIO or LED action, this determines how long the desired state is held (ie; how long the GPIO is held High or Low)
- Also Forward to USB; This option when enabled, still sends the original button action to the host via usb rather than blocking it (ie if you map VOL+ to trigger GPIO, you can choose to still send that command to the host machine)
- Use Hold Time; How long the user needs to hold the button down for the perscribed action to take place a.k.a long press button to invoke action.


### Advanced Menu

**WiFi Configuration**
- SSID and password management
- Connection status and IP address display


 **Other Features**
- User Settings backup and restore
- Factory reset 
- HID Descriptor cloning (some devices you may need to clone the BLE devices descriptor and forward to usb host)
- Connection LED animation toggle
- Factory reset option
- Minimum hold time (ms), this is to weed out accidental button presses but requiring all buttons to be held for X amount of time (this is separate to the delay timers in mapping options)

## Hardware

- ESP32-S3-DevKitC-1 N16R8 : https://s.click.aliexpress.com/e/_c3xrRIb5
- Airmouse M5 : https://s.click.aliexpress.com/e/_c4Lk3WAL

![Screenshot_20251109_170628_Samsung Internet](https://github.com/user-attachments/assets/ab70c232-050d-41ec-8904-34a026fbce75)
![Screenshot_20251109_170715_Samsung Internet](https://github.com/user-attachments/assets/7e5862ac-91f5-4d08-b4a5-c46c2399f5a7)
![Screenshot_20251109_170727_Samsung Internet](https://github.com/user-attachments/assets/c8b9a231-88df-4a19-8165-7c10b67535cd)



## Flashing Instructions

### Prerequisites 
- ESP32-S3 c1 connected to pc via the COM usb port not the one labled USB
- USB drivers installed (CH343) https://www.wch-ic.com/downloads/CH343SER_EXE.html

### Using the Flash Tool (Windows)

1. **Locate Files**
   - Navigate to `release` folder
   - You need: `HTPC-BLEBoot-Flasher.exe` and `HTPC-BLEBoot-ESP32S3.bin`

2. **Run Flash Tool**
   - Double-click `HTPC-BLEBoot-Flasher.exe`
   

3. **Select Firmware**
   - Click "Browse" button
   - Select `HTPC-BLEBoot-ESP32S3.bin`

4. **Select COM Port**
   - Choose your ESP32's COM port from dropdown
   - Click "Refresh" if device not listed
   - If not showing ensure the driver is installed and you have rebooted your pc, holding the *BOOT* button down while connecting the ESP32 may be required.

5. **Flash Firmware**
   - Click "Flash Firmware" button
   - Wait for completion (60+ seconds)

6. **Verify**
   - Device will reset automatically
   - LED will light up indicating successful boot

<img width="2560" height="1440" alt="{70E19FB4-CC4D-470D-81C9-7A170982CABF}" src="https://github.com/user-attachments/assets/fa3817a6-978e-4d93-a688-fe654d89977e" />


### Troubleshooting Flash Issues

**No COM Ports Detected**
- Install USB drivers 
- Try different USB cable or port

**Flash Failed**
- Hold BOOT button during "Connecting..." phase
- Lower baud rate to 115200 in Advanced Mode
- Verify correct chip type is selected

**Application Freezes**
- Close other programs using the COM port
- Wait for current operation to complete
- Check console output for detailed error messages

### First Time Setup

After successful flash:

1. Device creates WiFi access point: `HTPC-BLEBoot`
2. Connect to this network (no password)
3. Open browser to `http://192.168.4.1` or `remote.local`
4. Click Scan to find your BLE remote
5. Once Pairing is successful, the built in led with flash in a rainbow


### Alternative Flashing Methods

**esptool.py (Command Line)**
```bash
esptool.py --chip esp32s3 --port COM3 --baud 921600 write_flash 0x0 HTPC-BLEBoot-ESP32S3.bin
```

**PlatformIO**
- Open project folder
- Run: `platformio run --target upload`

**Arduino IDE**
- Select "ESP32-S3 Dev Module"
- Upload compiled binary via "Sketch > Upload"
