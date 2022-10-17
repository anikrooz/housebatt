**Code for ESP32 to manage VW 44.4v batteries**


Uses Vertiv / Emerson R48 Power Supply as the charger (included as submodule here; see below on cloning), SoyoSource GTN 1200 inverter via RS485, and interfaces with VW GTE battery packs via CAN bus. The ESP32 has one built-in CAN controller but this code uses MCP2515 transceivers to allow for multiple buses over shared SPI.

The job of this system is to respond to incoming MQTT messages of instantaneous in/out grid power and try to keep it to zero, by either charging or discharging the batteries. Code for energy meter publishing over MQTT in separate repository, but simply put this will subscribe to a tinyMQTT broker and expects a message every 500ms or so.

This has been designed for remote administration - it'll publish all sorts of data via MQTT so you can keep an eye on it and display nicely on homeassistant, and it uses the TelnetStream library to allow for remote commands and more detailed log following. Telnet is really simple - if you have the Windows feature turned on for example, just type "telnet <ip address>" in a command prompt and you'll get current status and a the keys to toggle debug modes, etc.

**Cloning**

Because I've published the charger manager as a separate repo, it's included as a submodule here. To clone this project locally, use:
`git clone --recurse-submodules`


**Credits**

https://github.com/Tom-evnut/VW-bms

https://github.com/hsaturn/TinyMqtt

https://github.com/syssi/esphome-soyosource-gtn-virtual-meter

https://github.com/ChrisHomewood/MQTT_to_Soyosource-Inverter_RS485

https://github.com/jandrassy/TelnetStream
