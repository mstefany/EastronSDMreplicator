# EastronSDMreplicator
Eastron SDM630 replicator / bridge / ...

## Origin
I have a small Solar Power Plant constructed from InfiniSolar 10 kW Hybrid Inverter, Hoepecke battery and Axitec solar panels.
Inverter uses Eastron SDM630 Modbus electricity power meter to measure feed-in to the grid and balance phases.
This is done via RS485 Modbus RTU protocol (InfiniSolar as Modbus client, Eastron as Modbus server). SDM630 measures various interesting values,
but these cannot be accessed since Modbus RTU is single-client multi-server only.
This prototype acts as a "man-in-the-middle":
 - reads all registers from Eastron SDM630 meter and stores them in local registers
 - these are then accessible to both InfiniSolar via separate Modbus RTU serial interface
   and Modbus TCP clients
