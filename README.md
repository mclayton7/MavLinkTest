# .NET MAVLink Control with simulator

# Sim Setup

## Running the Simulator on a Raspberry Pi 4
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py --mcast --no-mavproxy
```

## Resetting the Simulator
This wipes out any data stored on the fake autopilot's EEPROM and resets the data stream. After it starts, Ctrl+C to quit, and then run the simulator normally.
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```