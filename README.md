[![License: LGPL v3](https://img.shields.io/badge/License-LGPL_v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)
![License: LGPL v3](https://img.shields.io/badge/Embedded%20C-STM32-blue)

# STM32_FCU
### An stm32-based Flight Controller Unit
This is a voluntary ITEK summer project at OAMK, which is an independent summer project for students at Oulu University of Applied Sciences.

![Image Alt Text](./pictures/Pcb.png)

## Contents
 - 0.0 - What is a FCU?
 - 1.0 - Project goals
 - 2.0 - Components used
 - 3.0 - Software & flowchart
 - 4.0 - PCB layout
 - 5.0 - Issues and improvements
   
## 0.0 - What is a FCU?
A flight controller unit, an abbreviation for FCU, works as a general manager for a drone.
It's purpose is to handle communication between a drone's different components. The most basic FCU handles the task of transferring the commands it receives from a user radio to 4 PWM signals to be delivered to the Electronic Speed Controller. 

## 1.0 Project goals
The goal of this project is to design a flight controller that is able to receive commands from a radio, and turn them into useful signals for a commercial ESC. The drone also needs to be able to fly in a stable manner, meaning some sort of positional correction is to be needed.

This project uses a PDB(Power distribution board) to control 4 different ESCs. As illustrated in the picture below, the signal connectors from the ESCs are connected to this board.

## 2.0 - Main Components used
Here are the components chosen for this project:
- STM32F303RBT6 (Microcontroller). You can also use the STM32F303RCT6 with hardware generated PWM for ease of use.
- MMA8452Q Three-axis accelerometer for positional correction in mid-air(unimplemented as of yet.)
- NRF24L01 Radio transceiver for data transmission and receiving.

## 3.0 Software & flowchart
To make this project work, you need both the NRF24L01 Library as well as the MMA8452 library. The MMA library is included as a part of this project, and you can get the NRF library from here: 
https://github.com/controllerstech/NRF24L01.
Remember to customize the NRF library according to the device you are using. The original library was written for the F4-series, and we use the F3 in this project.

Here is the flowchart for this controller: 

![image](https://github.com/constlo/STM32_FCU/assets/79052688/8fafa1a4-70f2-4ed6-8835-1e89906d903e)

The packet that the controller receives should consist of the following bytes:

| Byte 0  | Byte 1 | Byte 2 | Byte 3 | Byte 4 | Byte 5 | Byte 6 | Byte 7 | Byte 8 | Byte 9 | Byte 10 |
| ------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- |
| Start byte valued at 204  | Settings byte. this will be used in the future. | Throttle MSB | Throttle LSB | Roll MSB | Roll LSB | Pitch MSB | Pitch LSB | Yaw MSB | Yaw LSB | End byte valued at 51. |

I will link the controller repository to this project later on, although the code used is very minimal.


## 4.0 Pcb Layout

![Image Alt Text](./pictures/Pcb_design.png)

Passive components:
- LDO: LM1085 3.3V
- Microcontroller: STM32F303RBT6 or - RCT6
- Capacitors: C1 to C7 - 100n decoupling capacitors, C8 and C9 are polcaps for LM1085 rated at 10u
- resistors: R1 should be roughly 10k to pull up the boot0 pin if needed. R2 to R8 are 150 ohm resistors for the PCB's leds.
- The crystal used is a 16 MHz crystal.

The PCB i designed mostly works, but a few changes had to be made after ordering the PCB:
- The LM1085 regulator's adjust pin must be shorted to ground with no resistance. (Resistor R2)
- There must be a THT resistor from the 3.3V terminal to the reset pin. I forgot the pull-up resistor, so this is a workaround for now.

## 5.0 Issues and improvements
Issues:
- ESCs may not turn on at the same time, 
leading to unbalanced flight or some of the motors not starting at all.
- Loose wires were an issue in ealy testing, leading to potential fire hazards due to stripped wires. 
- The control scheme is very primitive, and the analog stick values from the transmitter are very noisy.

Improvements for the future:
- VTX slot for the FCU
- Smaller package sizes to save PCB space as well as reducing noise and costs.
- Avoiding basic design mistakes, such as leaving the reset pin floating, resulting in errors.
