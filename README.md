# Embedded RTOS PID Controller for Hot Air Plant

This project implements an embedded controller for a hot air plant system using a uC/OS-based real-time operating system (RTOS) on the STM32F429 microcontroller. The controller performs PID feedback control for the plant, allowing both manual and automatic control modes. The system also includes a graphical user interface (GUI) to visualize the controller's performance and parameters.

## Project Overview

The purpose of this project was to develop a controller for a hot air plant using embedded systems principles, with a focus on implementing PID control and GUI functionality. The project leverages the uC/OS-III RTOS running on the STM32F429I-DISC1 microcontroller. The system features the following:

- **Manual and Auto Mode:** Allows the user to switch between manual and automatic control modes, each controlling different variables (voltage or temperature).
- **GUI:** Displays real-time plant parameters like temperature and setpoints on the STM32 display.
- **PID Control:** Implements PID feedback control to adjust the plant's output based on the setpoint and the current temperature.

## Background

The hot air plant system is used to model the behavior of a real-time control system. It consists of:

- **Monitoring:** Collecting data like air temperature and setpoint values.
- **Control:** Applying the PID algorithm for real-time temperature control.
- **Actuation:** Adjusting the voltage to the heater to control temperature output.

The PID algorithm was implemented using a discrete-time system, converted from the continuous-time transfer function of the hot air plant.

## Features

- **Manual Control:** Voltage setpoints are adjusted manually.
- **Auto Control:** Temperature setpoints are adjusted automatically.
- **Graphical Interface:** Real-time graphs of temperature vs. time, and dynamic display of setpoint and output variables.
- **Interactivity:** User inputs are taken through a keyboard (press 'a' for auto mode, 'm' for manual mode, and 'u'/'j' to adjust setpoints).

## Installation

1. Set up the STM32F429I-DISC1 hardware environment with the necessary tools for flashing the microcontroller.

2. Build the project using an IDE or build system compatible with STM32 development (e.g., STM32CubeIDE or Keil).

3. Flash the firmware to the STM32F429 board and run the program.

## Usage

- Press 'a' to switch to automatic mode.
- Press 'm' to switch to manual mode.
- Use the 'u' and 'j' keys to adjust the setpoint in either mode.
- The system will display the temperature and voltage on the STM32 display, updating the graph in real-time.

## Results and Discussion

- **Functionality:** The system successfully switches between manual and auto modes, and updates the display accordingly.
- **PID Control:** Initially, the PID control had oscillations due to improper proportional gain. Once tuned, the system tracked the temperature setpoint effectively.
- **Graphical Issues:** A trail appeared behind the setpoint line when moved too quickly, as the screen updates at a fixed 200ms interval.
- **Task Communication:** The system used static global variables for task communication, which could cause issues in a real-world implementation. Proper inter-task communication mechanisms (e.g., queues) should be used for safety and reliability in future projects.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

