# Raspi4b_stm32_CAN  
CAN bus communication between Raspberry Pi 4B and STM32  

## Overview  
This project demonstrates how to connect a **Raspberry Pi 4B** (via USB-to-CAN adapter) with an **STM32 MCU** using the **SN65HVD230 CAN transceiver**.  

## Hardware  
- Raspberry Pi 4B  
- USB-to-CAN adapter  
- STM32 microcontroller with CAN peripheral  
- SN65HVD230 CAN transceiver  

## Usage  
- Configure Raspberry Pi with `can-utils`  
- Enable CAN peripheral on STM32 (via CubeMX or HAL)  
- Test communication using `candump` and `cansend`  
