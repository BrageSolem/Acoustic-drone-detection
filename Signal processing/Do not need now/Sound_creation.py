
import numpy as np
from scipy.io import wavfile
import matplotlib.pyplot as plt
import pandas as pd

fs = 16000          # Hz
duration = 2.0     # s
frequency = 10000.0  # Hz 
amplitude = 0.8    # max 1 
t = np.linspace(0, duration, int(fs * duration))
"""
signal = ( # multi harmonic signal (from 200 to 800)
    0.6 * np.sin(2*np.pi*200*t) +
    0.3 * np.sin(2*np.pi*400*t) +
    0.1 * np.sin(2*np.pi*800*t)
)
"""
# Drone sim
signal = np.sin(2*np.pi*7000*t) + np.sin(2*np.pi*7100*t)+ np.sin(2*np.pi*7200*t)+ np.sin(2*np.pi*7300*t)+ np.sin(2*np.pi*7400*t)+ np.sin(2*np.pi*7500*t)
#modulation = 1 + 0.2*np.sin(2*np.pi*30*t) # amplitude variation
#modulation += 0.2 * np.sin(2*np.pi*4000*t) + 0.2 * np.sin(2*np.pi*6000*t) # additional high frec 
 
scale_white_noise = 0.5
white_noise = scale_white_noise * np.random.normal(scale=1.3, size = len(signal))
signal_noisy = signal + white_noise

signal_int16 = np.int16(signal_noisy / np.max(np.abs(signal)) * 32767)
wavfile.write("sim_drone.wav", fs, signal_int16)
sound_df = pd.DataFrame({"Raw sound" : signal_int16})
sound_df.to_csv("sudo_sound", index=False) # index=False prevents adding an index column

plt.figure(figsize=(10, 3))
plt.plot(signal_noisy[:3000], label="Noisy signal")
plt.plot(signal[:3000],  label="Clean signal")
plt.xlim(0,100)
plt.title("Time-domain waveform")
plt.xlabel("Sample index")
plt.ylabel("Amplitude")
plt.legend()
plt.tight_layout()
plt.savefig("Noisy_vs_clean_signal.png", dpi=300, bbox_inches="tight")
plt.show()



