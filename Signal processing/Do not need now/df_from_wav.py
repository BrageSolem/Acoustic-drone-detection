import librosa
import pandas as pd
import numpy as np
from scipy.signal import remez, freqz
from scipy.fft import rfft,fftfreq
import matplotlib.pyplot as plt
from scipy.io.wavfile import write

audio_file = "B_S2_D1_067-bebop_000_.wav"
signal, fs = librosa.load(audio_file, sr=20000)

print(signal)

mag_x_t = np.abs(rfft(signal))

plt.figure()
plt.stem(np.arange(len(mag_x_t)), mag_x_t, markerfmt="x")

M = 11
window = np.hanning(M)
white_noise_filter = window/window.sum()

#bandpass = remez(101, [0, 0.5,
#                     0.8, 8000,
#                     8001, fs/2],
#                     [0,1,0],
#                     fs = fs,
#                     weight=[1,100,1],)
#bandpass /= np.sum(bandpass)

#filter_coeffs = np.convolve(white_noise_filter, bandpass, "same")
signal_filtered = np.convolve(signal, white_noise_filter, "same").astype(np.float32)
mag_X_S = np.abs(rfft(signal_filtered))


plt.figure()
plt.stem(np.arange(len(mag_X_S)), mag_X_S,  markerfmt = "x")
plt.show()

write("B_S2_D1_067-bebop_000_filtered.wav", int(round(fs)), signal_filtered)
print("File created.")