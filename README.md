## Update 12/3-2019

New PCB version 3.0.8 added. Including gerber files for PCB in .zip file.


## Update 30/11-2018
Final slave module is ready and running. Software has been updated on slaves and master. Big packs of 100 cells, has been added. 
Pictures:
![Figure what](https://github.com/tvixen/diy-BMS-with-Arduino-Nano/blob/master/BMS_with_Nano-Circuit/Version%203.0.8/DSC_0835.JPG?raw=true "Figure")

## Update 13/7-2018

Arduino code for Master is uploaded. It's purly working as master, and without any debug information. But it should be plug'n play.
Arduino code for Slave is uploaded. No really debug information in code.


## Update 11/7-2018

Feel free to add comments or copy the whole project. It will be updated continuously.....
Next update will be the code for the Master and Slave.
Hardware drawings was updated to 3.0.4

-------------------------------------------------------------------------------------------------------------------------------

# Project "diyBMS with Arduino Nano" (11/7-2018)
Do it yourself battery management system to Lithium ion battery packs/cells
Inspired by Stuart Pittaway, and therefore the credit goes to him.

I was building a powerwall, and for that I needed a sort of batteri management. I have had a look at Batrium, but they was located in the US and the import would be to costly. So I found 
Stuart's project and thourght "This could work". But I was in need of a display and a precise 16bit ADC, so I found some cheap alternatives and added them to the circuit. A new PCB was now under construction.
Then I ordered the parts, and for my big surprise there were no power supply chips or ATtiny85 chips on the marked. So I decided to change the design and use what I had in the drawer. A handful of Arduino Nano.
So therefore this project is designed around the Arduino Nano with a boot sector from the Optiboot project. (no big deal of burning an ordinary Nano with Optiboot) 
The features are also a little different from the original project, as I’m not using the ESP8266 with Wifi, so there will be no transmitting data to a web interface.
Instead the slave modules can show the value of the battery pack (very precise), and the module number itself. The Master will show the status of all the modules, and also send balance commands 
to the module who is sticking out from rest of the packs/cells. + Every module will check if the voltage is over 4.200 volts and balance itself if needed. 




# The design

* One central controller and up to 24 individual cell monitoring nodes.
* The PCB is intented to be sitting in front of a batteri pack with 100.pcs 18650 cells, soldered together with fuses of 1.amp each.
* Use Arduino code, libraries and tools
* Use Arduino Nano as the Master and Slave, so a single PCB can be used as both Master or Slave. 
* Use good connection cables with low noise (twisted pairs)
* The Master will monitoring and control the slaves on the bus.
* The Slave will monitor itself, and discharge if the voltage is higer than 4.200 volts.
* Every single slave will show the pack voltage and the module number on the LED display.
* Ensure each cell voltage is isolated from other cells and that ground voltage is isolated
* Ensure communications between controller and node is isolated
* Keep the coste to a minimum and use whatever theres in the drawer.


# The code

* I did try to use as much of Stuart's origianal code as possible, but had to add a lot of stuff 
* keep it short and simple
* Try not to do inline stuff, as this can be difficult to convert to other platforms/chips
