import numpy as np
import matplotlib.pyplot as plt

# Define the timestamps for the signal
t1 = 2  # Time when the signal goes from 0 to 1
t2 = 5  # Time when the signal goes from 1 to 0
t_end = 10  # End time for the signal

# Generate time array
time = np.linspace(0, t_end, 1000)

# Create the signal
signal = np.zeros_like(time)
signal[(time >= t1) & (time <= t2)] = 1

# Plot the signal
plt.plot(time, signal, label="Signal")
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Signal from 0 to 1 and 1 to 0')
plt.grid(True)
plt.savefig("test_signal.png")