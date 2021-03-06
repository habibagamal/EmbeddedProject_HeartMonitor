import serial
import time
import threading
import sys
import matplotlib.pyplot as plt
import datetime as dt

plt.ion()
i=0
x=list()
y=list()

command = ""
produced = 0
lock = threading.Lock()

arduinoData = ""
off = 0
# ##############################
# ## thread reads command line input and sends data to microcontroller
# ##############################
class readThread (threading.Thread):
    def run (self):
        global command
        global x
        global y
        global off
        while (1):
            try:
                command = input()
                x = list()
                y = list()
                off = 1

                while (len(command) < 6):
                    command += 't'
                ser.write(command.encode())
            except KeyboardInterrupt:
                    print ("Interrupted by user")
                    sys.exit()        

port_name = input("Enter COM port name : ")
baud_rate = input("Enter baud rate : ") 

ser = serial.Serial(port_name, baudrate = baud_rate, timeout=1000000000)

reader = readThread()
reader.start()

# ##############################
# ## Plotting
# ##############################
while True:
    ser.flushInput()
    arduinoData = ser.readline().decode().strip()
    print(arduinoData)
    if(arduinoData.isdigit()):
        x.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        y.append(int(arduinoData))
        plt.cla()
        plt.xticks(rotation=45, ha='right')
        plt.subplots_adjust(bottom=0.30)
        plt.title('ECG Signals Over Time')
        plt.ylabel('ECG (voltage)')
        plt.plot(x,y)
        plt.show()
        plt.pause(0.0000001)
    