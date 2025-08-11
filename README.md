# STM32-CAN-Sim

## Overview
This project is a simulation framework for CAN (Controller Area Network) communication using STM32 microcontrollers. It consists of two main components:

1. **CAN_HMI**: A Human-Machine Interface (HMI) module for interacting with the CAN network.
2. **CAN_Motor**: A motor control module that simulates motor operations over the CAN network.

The project is developed using STM32CubeIDE and is based on the STM32F103C8 microcontroller.

## Project Structure

```
STM32-CAN-Sim/
├── CAN_HMI/
│   ├── Core/
│   │   ├── Inc/       # Header files
│   │   ├── Src/       # Source files
│   │   └── Startup/   # Startup assembly code
│   ├── Debug/         # Build outputs (binary, ELF, hex, etc.)
│   └── Drivers/       # HAL and CMSIS drivers
├── CAN_Motor/
│   ├── Core/
│   │   ├── Inc/       # Header files
│   │   ├── Src/       # Source files
│   │   └── Startup/   # Startup assembly code
│   ├── Debug/         # Build outputs (binary, ELF, hex, etc.)
│   └── Drivers/       # HAL and CMSIS drivers
└── README.md          # Project documentation
```

## Features
- **CAN Communication**: Simulates communication between HMI and motor modules over the CAN bus.
- **STM32 HAL**: Utilizes STM32 HAL (Hardware Abstraction Layer) for peripheral configuration and communication.
- **Modular Design**: Separate modules for HMI and motor simulation.

## Requirements
- **Hardware**: STM32F103C8 microcontroller or compatible board.
- **Software**: STM32CubeIDE for development and debugging.

## Getting Started

1. Clone the repository:
   ```
   git clone https://github.com/phatdoan301/STM32-CAN-Sim.git
   ```

2. Open the project in STM32CubeIDE.

3. Build and flash the firmware for both `CAN_HMI` and `CAN_Motor` modules.

4. Connect the modules via a CAN bus and power them up.

5. Monitor and interact with the system using the HMI module.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Author
Developed by [phatdoan301](https://github.com/phatdoan301).