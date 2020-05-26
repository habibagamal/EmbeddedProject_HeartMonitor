# HEART MONITOR
This project reads ECG signals using ECG sensor, plots those signals in real-time and computes the heart rate

## Directory Structure
- Milestone 1: contains the slides, the project and the readme for milestone 1
- KEIL_CubeMX_Projects: contains the project for the final demo
- Screenshots for BPM: includes 5 screenshots of calculated bpm and plot of ECG signals
- Videos for BPM: includes 2 videos (one with circuit shown and one without circuit) for process of collecting 1 min-worth of data and repoting bpm
- The C code is in /KEIL_CubeMX_Projects/Src
- The Python code is in /KEIL_CubeMX_Projects/MDK-ARM

- Supported commands: 
    - To collect 1 minute worth of data: "start"
    - To report bpm: "bpm"
    - To change the sampling frequency: "f=xxxx"
- To-do: 
    - Compute several values for bpm in 1 minute and take average

## To run final milestone
- Build and download the C program onto the STM32 microcontroller
- Make the appropriate connections between the USB-to-TTL and the micronctonroller 
- Make the appropriate connections between the ECG sensor and the microcontroller
Run these command at project root
```shell
cd KEIL_CubeMX_Projects/MDK-ARM/
python term.py
```

## To run milestone 1
- Build and download the C program onto the STM32 microcontroller
- Make the appropriate connections between the USB-to-TTL and the micronctonroller 
- Make the appropriate connections between the ECG sensor and the microcontroller
Run these command at project root
```shell
cd Milestone1/
python term.py
```
