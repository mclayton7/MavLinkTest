# .NET MAVLink Control with simulator

# Sim Setup

## Installation
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux

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

# Sim Operations

## Startup
1. Put the copter in ```GUIDED``` mode by typing the following into the prompt:
```
guided
```

2. Arm the motors:
```
arm
```

3. Takeoff:
```
takeoff
```

4. Loiters:
```
loiter-high
```

5. Land:
```
land
```

6. Disarm the motors:
```
disarm
```

# MavLink References
- [MavLink Serialization](https://mavlink.io/en/guide/serialization.html)
- [MavLink Messages](https://mavlink.io/en/messages/common.html)
- [MavLink Gimbal Control](https://mavlink.io/en/services/gimbal_v2.html)
- [MavLink C# Library](https://www.nuget.org/packages/MAVLink/)
- [ArduCopter Missions](https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html)
