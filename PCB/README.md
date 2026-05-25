# PCB and Electronics System

This folder contains the PCB files, schematics, and electronics documentation
for the ASV AeroCat V2. The board was designed to integrate the navigation
sensors, propulsion interfaces, power distribution, and communication system
into a single compact platform based on the ESP32 microcontroller.

Reference images of the schematic, PCB, and assembled electronics are available
in the [`images/`](images/) folder.

## System overview

The electronics system is responsible for:

- Navigation and orientation sensing
- Differential propulsion control
- Wireless communication
- Power regulation and distribution
- Signal isolation between control and propulsion systems

The PCB reduces the amount of external wiring by integrating the electrical
connections directly on the board.

---

# Propulsion system

The ASV AeroCat uses a differential aerial propulsion configuration composed
of two brushless propulsion units. Each unit contains a brushless motor,
an ESC, and a propeller.

Independent control of the left and right thrusters allows the vessel to
generate forward motion and turning maneuvers.

## Propulsion components

| Component | Specification | Quantity |
| --- | --- | ---: |
| ESC | Mistery 30A | 2 |
| Brushless motor | EMax CF2822 – 1200 kV | 2 |
| Propeller | 8x4.5” | 2 |

## Propulsion parameters

| Parameter | Value |
| --- | --- |
| Distance between propellers | 45 cm |
| Maximum surge force | 9.43 N |
| Maximum yaw torque | ±1.06 N·m |

---

# Navigation system

The navigation system is based on the ESP32 DevKit V2 platform and integrates
positioning, orientation, and inertial sensing modules directly into the PCB.

Compared to earlier versions that used jumper connections, the custom PCB
provides a cleaner organization and improves the integration between sensors,
actuators, and the control system.

## Navigation components

| Component | Function | Quantity |
| --- | --- | ---: |
| ESP32 DevKit V2 | Main controller | 1 |
| GY-NEO6M GPS | Global positioning | 1 |
| HMC5883L | Compass sensor | 1 |
| MPU6050 | Inertial measurement unit | 1 |

The GPS module provides positioning data, while the HMC5883L is used for
heading estimation. The MPU6050 is included for inertial sensing and future
sensor fusion applications.

---

# Communication system

The communication system uses the integrated wireless capabilities of the ESP32.

## Communication features

- Wi-Fi communication with the base station
- Bluetooth support for short-range operation
- Remote telemetry transmission
- Reception of navigation and control commands

During experimental operation, Wi-Fi was used as the primary communication
channel between the vessel and the base station.

---

# PCB integration

The PCB centralizes the electrical connections between sensors, propulsion
interfaces, power regulation, and control signals.

## Integrated components

| Component | Description | Quantity |
| --- | --- | ---: |
| ESP32 DevKit V2 | Main microcontroller board | 1 |
| GY-NEO6M | GPS module connector | 1 |
| HMC5883L | Compass connector | 1 |
| MPU6050 | IMU connector | 1 |
| PC817 | Optocoupler | 4 |
| 7805 | Voltage regulator | 1 |
| 1N5400 | Protection diode | 1 |
| Capacitor 100 pF | Signal and power filtering | 6 |
| Capacitor 1 µF | Voltage filtering | 1 |
| Resistor 330 Ω | Optocoupler input resistor | 4 |
| Resistor 10 kΩ | Pull-down resistor | 4 |
| ESC connectors | ESC signal outputs | 2 |
| Servo connectors | Servo outputs | 2 |
| Battery connector | Main power input | 1 |
| Power connector J1 | Auxiliary power connection | 1 |

## PCB functions

The PCB integrates:

- ESP32 communication and control interface
- GPS serial communication
- I2C communication for compass and MPU6050
- PWM outputs for ESC control
- PWM outputs for servo connections
- Battery power input
- 5 V voltage regulation
- Signal isolation using optocouplers
- Power filtering and stabilization

The schematic defines the relationship between the propulsion system,
navigation sensors, power electronics, and the ESP32 control platform.

---

# Electronics installation

All electronics should be installed inside the central module of the vessel.

## Installation guidelines

1. Install the PCB inside the electronics compartment.
2. Connect the GPS, compass, and MPU6050 modules to their headers.
3. Connect the ESC signal lines to the PCB outputs.
4. Connect the battery to the power input terminals.
5. Route the motor wires toward the propulsion supports.
6. Verify all power and signal connections before powering the system.
7. Ensure proper insulation and protection against humidity inside the module.

---
