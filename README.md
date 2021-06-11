# EastronMITM
Eastron SDM630 Modbus (Man) In The Middle

## Origin
I have a small Solar Power Plant constructed from InfiniSolar 10 kW Hybrid Inverter, Hoepecke battery and Axitec solar panels.
Inverter uses Eastron SDM630 Modbus electricity power meter to measure feed-in to the grid and balance phases.
This is done via RS485 Modbus RTU protocol (InfiniSolar as Modbus Master, Eastron as Modbus slave). SDM630 measures various interesting values,
but these cannot be access since Modbus is single-master multi-slave only.
This prototype acts as a "man-in-the-middle":
 - reads all registers from Eastron SDM630 meter and stores them in local registers
 - these are then accessible to both InfiniSolar via separate Modbus RTU serial interface
   and Modbus TCP clients
