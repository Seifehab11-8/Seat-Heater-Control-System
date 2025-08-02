# Seat Heater Control System

A real-time embedded system for controlling seat heaters using FreeRTOS on TM4C123GH6PM microcontroller.

## Features

- Real-time temperature monitoring using internal temperature sensor
- PWM-controlled heating element with multiple power levels
- User input handling for power and level control
- Safety monitoring with automatic shutdown on overheating
- LED status indicators for system state
- Multiple FreeRTOS tasks with priority-based scheduling

## System Architecture

### Tasks
- **Temperature Sensor Task** (Priority 2): Reads temperature every 100ms
- **Heater Control Task** (Priority 3): Controls PWM output based on temperature
- **User Input Task** (Priority 1): Handles button presses for power and level
- **Display Task** (Priority 1): Updates LED status indicators
- **Safety Monitor Task** (Priority 4): Monitors for fault conditions

### Hardware Interface
- **GPIO Port A**: User buttons (power, level control)
- **GPIO Port B**: Status LEDs (power, heating, fault)
- **PWM Module**: Heater control output
- **ADC Module**: Temperature sensor input

## Build Instructions

1. Install ARM GCC toolchain
2. Set up TivaWare driverlib
3. Configure FreeRTOS source paths
4. Build using make:
   ```
   make all
   ```

## Safety Features

- Automatic shutdown at 45°C
- Temperature monitoring with hysteresis
- Safety timer for critical overheating
- Fault indication via status LED

## Operation

1. Press power button to enable system
2. Press level button to cycle through heating levels
3. System automatically maintains target temperature
4. Safety system prevents overheating