import numpy as np
from scipy import signal
from collections import deque
import matplotlib.pyplot as plt

# Define the filter parameters
cutoff_frequency = 10.0  # Adjust this cutoff frequency as needed
nyquist_frequency = 0.5 * 10  # Nyquist frequency for a sampling rate of 1000 Hz
order = 4  # Filter order (adjust as needed)
window_size = 10  # Size of the buffer for filtered data

fc = 10.0  # Adjust as needed

# Design the filter with the new cutoff frequency
b, a = signal.butter(order, fc / nyquist_frequency, btype='low')

# Create an example function to simulate streaming sensor data
def generate_sensor_data(t):
    return 2 * np.sin(2 * np.pi * 5 * t) + 0.5 * np.random.normal()

filtered_data = deque(maxlen=window_size)  # Fixed-size buffer

# Initialize the filter state
zi = None

# Simulate the online data streaming process
for i in range(100):
    t = i * 0.01  # Time increment
    new_data_point = generate_sensor_data(t)

    if zi is None:
        zi = signal.lfiltic(b, a, [new_data_point])

    # Apply the Butterworth low-pass filter using lfilter
    filtered_data_point, zi = signal.lfilter(b, a, [new_data_point], zi=zi)

    # Append the filtered data to the deque
    filtered_data.append(filtered_data_point[0])

    # Use the filtered data in your control algorithm


    # Plot the filtered data and control output in real-time
    plt.plot(i, filtered_data_point[0], 'ro', markersize=5, label='Filtered Data')


    plt.pause(0.1)  # Pause to update the plot (adjust as needed)

plt.xlabel('Time Step')
plt.ylabel('Amplitude')
plt.legend()
plt.grid(True)
plt.title('Online Butterworth Low-Pass Filtering with lfilter')
plt.show()