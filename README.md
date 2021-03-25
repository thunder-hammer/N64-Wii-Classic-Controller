# N64-Wii-Classic-Controller
Use an N64 controller as a classic controller for the Wii

Designed to run on the Arduino Uno.

The current code is for a modified N64 controller with a wii nunchuck (resistive instead) joystick replacing the N64 joystick (wired direct to the Arudino) but can be modified to work with a stock N64 controller

The license file included is for the wiimote library.

Other things to note:

the wii mote supplies 3.3 volts out through the expansion port while the arduino requires 5 volts. In my case I found that 3.3 volts was sufficent for one of my arduino compatible boards, but not the other. a boost converter might do the trick to fix this but be careful as the n64 requires 3.3 volts. Also be careful plugging in 5 volts while programming over usb. This might damage the N64 controller, wiimote, or both.

Lastly:

Some of the arduino uno analog pins double as I2C pins. I2C can really mess with analog reads and putting enough capacitance on the analog lines to smooth it out will mess with I2C.
