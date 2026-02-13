
import serial
import struct
import numpy as np
import pandas as pd
import time
from scipy.io.wavfile import write
import matplotlib.pyplot as plt

PORT = "COM5"
BAUD = 921600
SYNC = b'\x5A\xA5'


# data amount i got with UART
# Sample output = 16 bit or 2 bytes
# 23.4k samples /s * 2 ch * 2 bytes = 93,6 kBytes/s * 8 bits =  ca 0,75 Mbit/s

# max allowed data through UART
# baud = 921600 - or bit of information
# 1 start bit, 8 databit, 1 stop bit
# so 10 bits transmitted per 8-bit signal
# 921600/10 = 92,160 bytes/s * 8 bits = 737,280 bit = 0,737 Mbit/s

# 0,737 / 0,75 ca 98 % efficient. 

# I suppose I could go as high as 2M baud, but that would allow me to have max up to 4 channels
# I presume it might be necessary to increase the number of channel to up to 8 or maybe even 10 microphone,
# for the tracking to be more accurate at closer distances.
# Hence ill try UDB CDC in other py file. 

CHANNELS = 2 # was 4 
DURATION_SEC = 10

ser = serial.Serial(PORT, BAUD, timeout=1)

samples_mic1 = []
samples_mic2 = []
samples_mic3 = []
samples_mic4 = []

start_time = time.time()

print("Recording...")

while time.time() - start_time < DURATION_SEC:
    # sync search
    if ser.read(1) != SYNC[:1]:
        continue
    if ser.read(1) != SYNC[1:]:
        continue

    # read one frame, sends 16 samples/bits per channel
    FRAME_SAMPLES = 16
    raw = ser.read(FRAME_SAMPLES * CHANNELS * 2)
    if len(raw) != FRAME_SAMPLES * CHANNELS * 2:
        continue

    data = np.frombuffer(raw, dtype=np.int16)

    # de-interleave
    samples_mic1.extend(data[0::2])
    samples_mic4.extend(data[1::2])
    """
    samples_mic1.extend(data[0::4])
    samples_mic2.extend(data[1::4])
    samples_mic3.extend(data[2::4])
    samples_mic4.extend(data[3::4])
"""
ser.close()
print("Done recording")

# Convert to numpy
mic1 = np.array(samples_mic1, dtype=np.int16)
#mic2 = np.array(samples_mic2, dtype=np.int16)
#mic3 = np.array(samples_mic3, dtype=np.int16)
mic4 = np.array(samples_mic4, dtype=np.int16)

# Create DataFrame (ML-ready)
df = pd.DataFrame({  #"mic2": mic2,"mic3": mic3,
    "mic1": mic1,
    "mic4": mic4,
})

df.to_csv('mic_recording_08_02_1.csv', index=False)

print(df.head())
print(f"Total samples per channel: {len(mic1)}")

plt.figure()
plt.subplot(2,1,1)
plt.plot(mic1[:2000])
plt.title("CH1 waveform")
#plt.subplot(4,1,2)
#plt.plot(mic2[:2000])
#plt.title("CH2 waveform")
#plt.subplot(4,1,3)
#plt.plot(mic3[:2000])
#plt.title("CH3 waveform")
plt.subplot(2,1,2)
plt.plot(mic4[:2000])
plt.title("CH4 waveform")
plt.show()