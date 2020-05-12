## To run this milestone
- Build and download the C program onto the STM32 microcontroller
- Make the appropriate connections between the USB-to-TTL and the micronctonroller 
- Make the appropriate connections between the ECG sensor and the microcontroller
Run these command at project root
```shell
cd Milestone1/
python term.py
```
-Supported commands: 
    - To collect 1 minute worth of data: "startt"
    - To report bpm: "bpmmmm" (microcontroller will UART trasnmit "report" to indicate the receipt of the command)
    - To change the sampling frequency: "f=xxxx"
To-do: 
- Auto-concatenate white spaces to short commands like "start" and "bpm" 
- Caculate and report bpm