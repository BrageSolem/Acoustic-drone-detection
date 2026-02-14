import pandas as pd
import numpy as np
from scipy.io.wavfile import write
import matplotlib.pyplot as plt
from scipy.signal import remez, freqz
from scipy.fft import fft,fftfreq

raw_signal = pd.read_csv("mic_recording_stereo_df.csv")

ch1 = raw_signal["ch1"].to_numpy(dtype=np.int16)
ch2 = raw_signal["ch2"].to_numpy(dtype=np.int16)

fs = int(len(ch1)/10) # change 60 when duration is changed

raw_ch1_mag = fft(ch1)
raw_ch2_mag = fft(ch2)
N = len(ch1)
Ts = 1/fs
freq = fftfreq(N,Ts)

plt.figure(figsize=(10,3))
plt.subplot(2,1,1)
plt.stem(freq[0:(N//2)], raw_ch1_mag[0:(N//2)], markerfmt="x")
plt.legend()
plt.subplot(2,1,2)
plt.stem(freq[0:(N//2)], raw_ch2_mag[0:(N//2)], markerfmt="x")
plt.legend()


M = 25
window = np.hanning(M)
white_noise_filter = window/window.sum()

bandpass = remez(101, [0, 0.5,
                     0.8, 8000,
                     8001, fs/2],
                     [0,1,0],
                     fs = fs,
                     weight=[1,100,1],)
bandpass /= np.sum(bandpass)

filter_coeffs = np.convolve(white_noise_filter, bandpass, "same")

filtered_ch1 = np.convolve(ch1, filter_coeffs, "same")
filtered_ch2 = np.convolve(ch2, filter_coeffs, "same")

filtered_ch1 = np.clip(filtered_ch1, -32768, 32767).astype(np.int16)
filtered_ch2 = np.clip  (filtered_ch2, -32768, 32767).astype(np.int16)

gain = 1
filtered_ch1 *= gain
filtered_ch2 *= gain

f_filter, H_filter = freqz(filter_coeffs, fs =fs )
filtered_ch1_mag = np.abs(fft(filtered_ch1))
filtered_ch2_mag =np.abs( fft(filtered_ch2))

stereo = np.stack((filtered_ch1, filtered_ch2), axis=1)
write("gained_filtered_mic_recording_stereo.wav", int(round(fs)), stereo)
print("File created.")


filtered_df = pd.DataFrame({
    "Filtered ch1": filtered_ch1,
    "Filtered ch2": filtered_ch2
})

filtered_df.to_csv('filtered_mic_recording_stereo_df.csv', index=False)

plt.figure()
plt.subplot(2,1,1)
plt.plot(ch1)
plt.title("CH1 waveform")
plt.subplot(2,1,2)
plt.plot(filtered_ch1)
plt.title("Filtered CH1 waveform")

plt.figure()
plt.subplot(2,1,1)
plt.plot(ch2)
plt.title("CH2 waveform")
plt.subplot(2,1,2)
plt.plot(filtered_ch2)
plt.title("Filtered CH2 waveform")

plt.figure()
plt.plot(f_filter,np.abs(H_filter), label ="White noise filter response ")
plt.title("Magnitude ")
plt.xlabel("Frequency [Hz]")
plt.legend()

plt.figure(figsize=(10,3))
plt.subplot(2,1,1)
plt.stem(freq[0:(N//2)], filtered_ch1_mag[0:(N//2)], markerfmt="x", label="filtered mag ch1")
plt.legend()
plt.subplot(2,1,2)
plt.stem(freq[0:(N//2)], filtered_ch2_mag[0:(N//2)], markerfmt="x", label="filtered mag ch2")
plt.legend()
plt.show()

