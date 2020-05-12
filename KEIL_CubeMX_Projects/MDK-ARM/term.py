import serial
import time
import threading
import sys
import matplotlib.pyplot as plt
import datetime as dt

plt.ion()
#fig = plt.figure()
i=0
x=list()
y=list()

command = ""
produced = 0
lock = threading.Lock()

arduinoData = ""
off = 1
# ##############################
# ## Producer thread
# ##############################
class producerThread (threading.Thread):
    def run (self):
        global command
        global produced
        while (1):
            if produced == 0:
                lock.acquire()
                try:
                    command = input()
                    produced = 1
                    lock.release()
                except KeyboardInterrupt:
                        print ("Interrupted by user")
                        lock.release()
                        sys.exit()        
# ##############################                
# ## Consumer thread
# ##############################
class consumerThread (threading.Thread):
    def run (self):
        global command
        global produced
        while (1):
            if produced == 1:
                lock.acquire()
                ser.write(command.encode())
                produced = 0
                lock.release()
# ##############################
# ## Printing thread
# ##############################
class printingThread (threading.Thread):
    def run (self):
        global arduinoData
        global off
        while (1):
            ser.flushInput()
            arduinoData = ser.readline().decode().strip()
            if (arduinoData != ""):
                off = 0
                print (arduinoData)
            else:
                arduinoData = ""
                off = 1

port_name = input("Enter COM port name : ")
baud_rate = input("Enter baud rate : ") 

ser = serial.Serial(port_name, baudrate = baud_rate, timeout=1000000000)

producer = producerThread()
producer.start()

consumer = consumerThread()
consumer.start()

printing = printingThread()
printing.start()

# ##############################
# ## Plotting
# ##############################
while True:
    if(arduinoData.isdigit() and off == 0):
        x.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
        y.append(int(arduinoData))

    xs = x[-20:]
    ys = y[-20:]

    plt.cla()
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('ECG Signals Over Time')
    plt.ylabel('ECG (voltage)')
    plt.plot(xs,ys)
    plt.show()
    plt.pause(0.0000001)



# ##############################
# ## Working plotting
# ##############################
# from drawnow import *

# values = []

# plt.ion()
# cnt=0
# def plotValues():
#     plt.title('Serial value from Arduino')
#     plt.grid(True)
#     plt.ylabel('Values')
#     plt.plot(values, 'rx-', label='values')
#     plt.legend(loc='upper right')
#     plt.show()
# for i in range(0,26):
#     values.append(0)

# while 1:
#     if (arduinoData.isdigit()):
#         values.append(int(arduinoData))
#         values.pop(0)
#         drawnow(plotValues)
# ##############################
# ## DONE Working plotting
# ##############################
    

