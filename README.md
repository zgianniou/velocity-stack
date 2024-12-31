# RPM-Controlled Adjustable Velocity Stack

This repository provides the code and documentation for an **adjustable velocity stack** system designed to enhance engine performance by optimizing the torque-RPM curve. The system uses an engine's RPM measurement to dynamically adjust the length of the velocity stack in real time.

## 🚀 Features
- **Real-Time RPM Measurement**: Uses a crankshaft position sensor to calculate RPM with high precision.
- **Dynamic Stack Adjustment**: Actuates the velocity stack length based on RPM to optimize airflow and enhance torque at various engine speeds.
- **Debouncing Logic**: Filters signal noise for accurate RPM detection.
- **User-Controlled Modes**: Supports configurable modes to adapt the velocity stack operation for different use cases.

## 🛠️ Hardware Requirements
- **Microcontroller**: ESP32 or compatible microcontroller with sufficient GPIO pins.
- **Crankshaft Position Sensor**: Supports Hall-effect or inductive sensors.
- **Actuator**: Linear actuator or servo motor for stack adjustment.
- **Power Supply**: 12V DC system (common in motorcycles).
- Optional:
  - Oscilloscope for signal analysis.
  - Voltage divider or signal conditioning circuit for safe sensor integration.

## 📝 How It Works
1. **RPM Measurement**:
   - A crankshaft position sensor generates pulses corresponding to the engine speed.
   - The ESP32 calculates RPM by measuring the time between consecutive rising edges of the signal.

2. **Actuator Control**:
   - Based on the measured RPM, the system determines the optimal stack length using a pre-defined torque curve mapping.
   - The velocity stack length is adjusted using a linear actuator or servo motor.

3. **Debounce Logic**:
   - Ensures accurate edge detection by filtering out noise and spurious signals.

## ⚙️ Setup Instructions
1. **Hardware Connections**:
   - Connect the crankshaft position sensor to the microcontroller's interrupt pin.
   - Wire the actuator (e.g., servo motor or linear actuator) to the appropriate control pin.
   - Ensure proper grounding and shielding for the sensor and actuator.

2. **Software Setup**:
   - Clone this repository:
     ```bash
     git clone https://github.com/Panther-Racing-AUTh/velocity-stack.git
     cd velocity-stack
     ```
   - Open the `src` folder in the Arduino IDE or PlatformIO.
   - Upload the code to the ESP32.

3. **Testing**:
   - Use the onboard serial monitor to verify RPM measurements and actuator movements.

## 📊 Performance Testing
- Include data visualizations or torque-RPM curves showing the effect of the adjustable velocity stack.
