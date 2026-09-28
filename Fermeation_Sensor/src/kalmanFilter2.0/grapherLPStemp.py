import pandas as pd
import matplotlib.pyplot as plt

# 1. Load the data (replace 'data.csv' with your actual filename)
df = pd.read_csv('kalmanFilter2.0/csvFiles/expJul13MOD.csv')

# 2. Convert the 'timestamp' column to proper datetime objects
df['timestamp'] = pd.to_datetime(df['timestamp'])

# 3. Calculate the new time axis
# This subtracts the 'uptime_ms' from the 'timestamp' as requested.
df['calculated_time'] = df['timestamp'] - pd.to_timedelta(df['uptime_ms'], unit='ms')

# IMPORTANT NOTE ON TIMING:
# If 'timestamp' was recorded at the VERY END of the run, and 'uptime_ms' 
# counts forward from 0 at the start of the run, subtracting uptime from the 
# end timestamp will actually reverse your timeline. 
# If you want the true chronological time of each reading, you would instead 
# subtract the *remaining* uptime from the end timestamp, like this:
# max_uptime = df['uptime_ms'].max()
# df['calculated_time'] = df['timestamp'] - pd.to_timedelta(max_uptime - df['uptime_ms'], unit='ms')

# 4. Create the plot
plt.figure(figsize=(10, 6))
plt.plot(df['calculated_time'], df['lps33hw_tempC'], 
         marker='o', linestyle='-', color='b', label='LPS33HW Temp')

# 5. Format the graph for readability
plt.title('LPS33HW Temperature Over Time')
plt.xlabel('Calculated Time')
plt.ylabel('Temperature (°C)')
plt.grid(True, linestyle='--', alpha=0.7)
plt.xticks(rotation=45) # Rotate timestamps so they don't overlap
plt.legend()
plt.tight_layout()      # Adjust layout to prevent clipping of labels

# 6. Display the plot
plt.show()