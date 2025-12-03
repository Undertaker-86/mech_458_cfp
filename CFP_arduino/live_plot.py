import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import sys
import time

# Configuration
FILENAME = "test.csv" # Default, can be overridden
MAX_POINTS = 500      # Number of points to show on graph

print("--- LIVE PLOTTER ---")
user_file = input(f"Enter filename to watch (default '{FILENAME}'): ")
if user_file:
    if not user_file.endswith(".csv"): user_file += ".csv"
    FILENAME = user_file

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle(f"Live Data: {FILENAME}")

def animate(i):
    try:
        # Read CSV (handle potential read errors if file is being written)
        try:
            df = pd.read_csv(FILENAME)
        except pd.errors.EmptyDataError:
            return
        except FileNotFoundError:
            print("Waiting for file...")
            return

        if df.empty: return

        # Keep only recent data
        df = df.tail(MAX_POINTS)
        
        # Calculate Time in Seconds
        if 'Time_ms' in df.columns:
            start_time = df['Time_ms'].iloc[0]
            time_sec = (df['Time_ms'] - start_time) / 1000.0
        else:
            return

        # Plot 1: Positions
        ax1.clear()
        ax1.plot(time_sec, df['Target'], 'k--', alpha=0.5, label='Target')
        ax1.plot(time_sec, df['M1_Pos'], 'b-', label='Master (M1)')
        ax1.plot(time_sec, df['M2_Pos'], 'r-', label='Slave (M2)')
        ax1.set_title("Position")
        ax1.set_ylabel("Encoder Counts")
        ax1.legend(loc='upper left')
        ax1.grid(True)

        # Plot 2: Sync Error
        ax2.clear()
        sync_error = df['M1_Pos'] - df['M2_Pos']
        ax2.plot(time_sec, sync_error, 'm-', label='Error (M1-M2)')
        ax2.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax2.set_title("Sync Error")
        ax2.set_ylabel("Counts")
        ax2.set_xlabel("Time (s)")
        ax2.legend(loc='upper left')
        ax2.grid(True)

    except Exception as e:
        print(f"Plot Error: {e}")

ani = animation.FuncAnimation(fig, animate, interval=500) # Update every 500ms
plt.tight_layout()
plt.show()
