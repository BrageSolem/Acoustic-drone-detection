import matplotlib.pyplot as plt
import pandas as pd
from scipy.io.wavfile import write
import numpy as np

df = pd.read_csv("mic_recording_usb.csv")
print(df.head())

mic1 = df["mic1"].to_numpy(dtype=np.int16)
mic2 = df["mic2"].to_numpy(dtype=np.int16)
mic3 = df["mic3"].to_numpy(dtype=np.int16)
mic4= df["mic4"].to_numpy(dtype=np.int16)
effective_fs = int(len(mic1)/4) # change 4 when duration is changed

stereo = np.stack((mic1, mic2, mic3, mic4), axis=1)
#stereo = np.stack((mic1, mic4), axis=1)

write("mic_recording_usb.wav", int(round(effective_fs)), stereo)
print("File created.")