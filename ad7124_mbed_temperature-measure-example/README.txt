Evaluation Boards/Products Supported
------------------------------------ 
EVAL-AD7124

Overview
--------
These code files provide the temperature measurement console application and device libraries to 
interface with AD7124 product evaluation board. This code was developed and tested on SDP-K1 
controller board: https://os.mbed.com/platforms/SDP_K1/

Product details: https://www.analog.com/en/products/ad7124-8.html, https://www.analog.com/en/products/ad7124-4.html
Product Evaluation board details: https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD7124-8.html
                                  https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-AD7124-4.html
User Guide for this code: https://wiki.analog.com/resources/tools-software/product-support-software/ad7124_temperature_measurement_demo
Communication Protocol: SPI


Hardware Setup
--------------
Required: SDP-K1 (or alternative Mbed enabled controller board), EVAL-AD7124
USB cable.
Plug in the EVAL-AD7124 board on SDP-K1 board (or any other Mbed enabled controller board) 
using the SDP or Arduino on-board connector (default set in software is Arduino).
Connect SDP-K1 board to the PC using the USB cable.


How to Get Started
------------------
Open Mbed online compiler. https://ide.mbed.com/compiler
Import Code into compiler from here: https://os.mbed.com/teams/AnalogDevices/code/EVAL-AD7124-Temperature_measurement/
instructions on how to import code are here: https://os.mbed.com/docs/mbed-os/v5.12/tools/importing-code.html
Compile code. Drag and drop binary into SDP-K1 controller board. Find detailed 
instructions here: https://os.mbed.com/docs/mbed-os/v5.12/tools/getting-your-program-on-your-board.html
Open Tera Term (or alternative), select 115200 baud rate, and the applicable COM port to see the list of options.


Notes
-----
If using Win 7, install serial drivers for Mbed. https://os.mbed.com/docs/mbed-os/v5.12/tutorials/windows-serial-driver.html
A detailed user guide on SDP-K1 controller board is available here https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/SDP-K1.html.


Copyright (c) 2020 Analog Devices, Inc.  All rights reserved.

