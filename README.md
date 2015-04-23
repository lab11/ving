Rumblecation
=======

![Vibratory Communication Device](https://raw.githubusercontent.com/lab11/rumblecation/master/media/rumblecation.JPG)

Mobile phones have vibratory motors which can transmit vibration, and 
accelerometers, which can sense vibration. Rumblecation takes advantage
of this hardware to communicate over vibration. Vibratory communication
would limit the communication domain to two or more physically coupled
devices, and this could be used to pair a specific subset of devices
sharing a surface such as table. 

Custom hardware was developed to closely emulate the hardware found in
common smartphones. A simple protocol was then designed for vibratory
communication and firmware was developed for our hardware to implement
this protocol.

Current work is focused on improving data rate and reliability, which
should be achievable by improving the algorithms that are processing 
accelerometer data.
