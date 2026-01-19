# Pump-Dispensing-Test-Setup-Automation

## Overview
This project implements a robust embedded automation system for a multi-pump dispensing test setup using a microcontroller. The system controls four pumps, a relay, and solenoid valves, providing real-time operator control through a menu-driven LCD interface. Key features include calibration, data logging, power-cut recovery, and precise pump operation, making it reliable and autonomous in industrial environments.

## Key Features
- **Multi-Pump Control:** Brine Pump, Hydra Pump, Submersible Pump, and BDD Pump with individual or all-pump operation.  
- **Relay & Solenoid Management:** Automated valve switching and relay logging for maintenance tracking.  
- **Menu-Driven LCD Interface:** Intuitive GUI for start/stop, manual control, calibration, and log viewing.  
- **Pump Calibration:** PWM-based output calibration (0–100%) stored in EEPROM for persistent settings.  
- **Power-Cut Recovery:** System state autosaved every 10 seconds; resumes operation after unexpected power loss.  
- **Data Logging:** Runtime and relay counts recorded to SD card for traceability and analysis.  
- **Button Debouncing & Safety:** Reliable push-button inputs and safety checks to prevent unintended operation.  

## Technologies Used
- Microcontroller (Arduino/Teensy compatible)  
- Embedded C++  
- I²C 16×2 LCD Display with push-button interface  
- EEPROM for persistent state storage  
- PWM for pump calibration  
- SD card data logging  

## System Workflow
1. **Main Menu**: Navigate between All Pumps, Manual, Calibration, and Log modes.  
2. **All Pumps Mode**: Start or stop all pumps together.  
3. **Manual Mode**: Control individual pumps or relays.  
4. **Calibration Menu**: Adjust pump output percentages (0–100%) and save in EEPROM.  
5. **Log View**: Display cumulative runtime of pumps and relay counts in hours:minutes.  
6. **Power-Cut Recovery**: Autosave system state every 10 seconds to EEPROM; on restart, system resumes previous state.  

## Benefits
- Ensures **reliable autonomous operation** even during power interruptions.  
- Provides **precise control** over pump flow rates.  
- Reduces **manual supervision** with automated logging and calibration.  
- Improves **maintenance planning** via relay count and runtime tracking.  

## Example Output
- LCD Menu Screens for Main Menu, Manual Control, Calibration, and Logs.  
- SD card logs capturing runtime and relay activity.  
- Smooth, reliable operation of all pumps and solenoids with PWM calibration.

## Usage
1. Connect pumps, relay, solenoids, LCD, and buttons according to the project wiring.  
2. Upload the firmware to a compatible microcontroller (Arduino/Teensy).  
3. Navigate the LCD menu to start pumps, calibrate, or view logs.  
4. System automatically handles autosave and resumes after power interruptions.  

## Impact
This system improves industrial automation reliability, reduces manual monitoring, and allows flexible operation across different dispensing recipes. It demonstrates integration of embedded C++ programming, hardware interfacing, GUI design, and industrial-grade automation principles.
