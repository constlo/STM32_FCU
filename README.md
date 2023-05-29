# STM32_FCU
### An stm32-based Flight Controller Unit
This is a voluntary ITEK summer project at OAMK, which is an independent summer project for students at Oulu University of Applied Sciences.

## Contents
0.0 - What is a FCU?
1.0 - Project goals
2.0 - Components used

# 0.0 - What is a FCU?
A flight controller unit, an abbreviation for FCU, works as a general manager for a drone.
It's purpose is to handle communication between a drone's different components. The most basic FCU handles the task of transferring the commands it receives from a user radio to 4 PWM signals to be delivered to the Electronic Speed Controller. 

# 1.0 Project goals
The goal of this project is to design a flight controller that is able to receive commands from a radio, and turn them into useful signals for a commercial ESC. The drone also needs to be able to fly in a stable manner, meaning some sort of positional correction is to be needed.

# 2.0 - Components used
Here are the components chosen for this project:
- STM32F4-series processor
- LMU500-series 3-axis gyroscope
- Connectors for the ESC, VTX and RTX

For this project the STM32 platform was chosen, for both familiarity to me, as well as it's robustness. STM32 is also the most commonly-chosen platform for commercial flight controllers, meaning they are a great choice for this project.

# 3.0 Basic software principle
The flight controller for a drone must keep the drone leveled when the user does not control the roll, pitch or yaw of a drone. To achieve this, a gyroscope must be used to determine the position of the drone. When the user inputs any of the three controls, this control must be overwritten, meaning the drone will output PWM to the drone according to the radio commands.
