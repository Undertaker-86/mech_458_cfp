import pandas as pd
import matplotlib.pyplot as plt
import sys

# ==== LOAD ====
print("--- PID TUNER & PLOTTER ---")
filename = input("Enter filename to plot (e.g. 'test1'): ")
if not filename.endswith(".csv"): filename += ".csv"

try:
    df = pd.read_csv(filename)
except FileNotFoundError:
    print("File not found.")
    sys.exit()

# ==== PROCESS ====
# Time in seconds
df['Time_Sec'] = (df['Time_ms'] - df['Time_ms'].iloc[0]) / 1000.0
# Sync Error (Master - Slave)
df['Sync_Error'] = df['M1_Pos'] - df['M2_Pos']

# ==== PLOT ====
fig, axs = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle(f'Analysis: {filename}', fontsize=16)

# 1. Position
axs[0, 0].scatter(df['Time_Sec'], df['Target'], c='k', alpha=0.5, label='Target')
axs[0, 0].scatter(df['Time_Sec'], df['M1_Pos'], c='b', label='M1')
axs[0, 0].scatter(df['Time_Sec'], df['M2_Pos'], c='r', label='M2')
axs[0, 0].set_title('Position')
axs[0, 0].legend()
axs[0, 0].grid(True)

# 2. Sync Error (TUNE PID HERE)
axs[0, 1].scatter(df['Time_Sec'], df['Sync_Error'], c='purple')
axs[0, 1].axhline(0, color='k', linestyle='--')
axs[0, 1].set_title('Sync Error (M1 - M2)')
axs[0, 1].set_ylabel('Pulses Difference')
axs[0, 1].grid(True)

# 3. RPM
axs[1, 0].scatter(df['Time_Sec'], df['M1_RPM'], c='b', label='M1')
axs[1, 0].scatter(df['Time_Sec'], df['M2_RPM'], c='r', label='M2')
axs[1, 0].set_title('Speed (RPM)')
axs[1, 0].legend()
axs[1, 0].grid(True)

# 4. PWM
axs[1, 1].scatter(df['Time_Sec'], df['PWM1'], c='b', label='M1')
axs[1, 1].scatter(df['Time_Sec'], df['PWM2'], c='r', label='M2')
axs[1, 1].set_title('PWM Effort')
axs[1, 1].legend()
axs[1, 1].grid(True)

plt.tight_layout()
plt.show()
