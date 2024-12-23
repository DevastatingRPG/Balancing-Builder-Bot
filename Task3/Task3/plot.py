import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for port in ports:
    print(port.device)

# Set up the Bluetooth serial port
ser = serial.Serial('COM8', 9600)  # Replace 'COM3' with your Bluetooth serial port

# Set up the plot
fig, ax = plt.subplots()
xdata, ydata1, ydata2, ydata3 = [], [], [], []
ln1, = plt.plot([], [], 'r-', label='Input_Pitch')
ln2, = plt.plot([], [], 'b-', label='Error_Pitch')
ln3, = plt.plot([], [], 'g-', label='Output_Pitch')

def init():
    ax.set_xlim(0, 100)
    ax.set_ylim(-90, 90)
    return ln1, ln2, ln3

def update(frame):
    line = ser.readline().decode('utf-8').strip()
    if line:
        data = line.split(',')
        if len(data) == 3:
            xdata.append(frame)
            ydata1.append(float(data[0]))
            ydata2.append(float(data[1]))
            ydata3.append(float(data[2]))
            ln1.set_data(xdata, ydata1)
            ln2.set_data(xdata, ydata2)
            ln3.set_data(xdata, ydata3)
            ax.relim()
            ax.autoscale_view()
    return ln1, ln2, ln3

ani = animation.FuncAnimation(fig, update, frames=range(100), init_func=init, blit=True)
plt.legend()
plt.show()