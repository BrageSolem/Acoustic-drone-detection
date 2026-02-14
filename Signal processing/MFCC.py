import numpy as np
import librosa
import librosa.display
import matplotlib.pyplot as plt
import pandas as pd


#audio_file = "B_S2_D1_067-bebop_000_.wav"   # replace with your file
audio_file = "mic_recording_04_02_1.wav"

#https://librosa.org/doc/main/generated/librosa.feature.mfcc.html
# to some degree based on

signal, fs = librosa.load(audio_file, sr=20000) 
# Sample much higher than max frequency if spectral accuracy matters

print("fs:", fs)
print("signal len:", len(signal))

"""
MFCC in a nutshell based on the "An Approach to Extract Feature using MFCC" (For the report):

1 .The input signal is gathered and sent to the pre-emphasis. The signal used in the current design is comprised of four 1D-signals, making it a 4D signal

2. Pre-Emphasis - increases the signal of higher frequencies by using a first order diff high pass filter

3. Frame blocking - basically means a division of the signal into smaller frames of f.eks 20 to 30ms each. Since the sound signal 
is varying over time, to be able to correctly utilize the FFT to capture present frequencies, the signal has to be stationary.

4. windowing - is used to reduce the effects of sudden changes in the state, which in the defult rectangle window, suddenly moves the signal from 0 to 1 in a sharp move.
The application of f.ex a hamming window smooths the transition reducing the side lobes, or the spectral leckage of the signal to other not necesarly present frequency bins.   
# Understanding Digital Signal Processing - book, pdf page 100, Figure 3.15

5. FFT - "moves" the signal from the time domain to the freq domain, obtaining the energy magnitude of the
present frequency components in the signal. The FFt is applied to each frame.

6. Triangular band pass - smooths the magnitude spectrum and reduces the size of features involved in the sepctral envelope.

7. Discrete cosine transform - is applied to obtain the mel-scale cepstral coeffs

7.1 Log energy - the energy within each frame can be calculated, adding a additional feature to the MFCC
7.2 Delta - may derive the features to obtain the velocity and acceleration of the spectral envelope

"""



# Pre-emphasis
# the func from librosa basically uses a first order diff filter https://librosa.org/doc/main/generated/librosa.effects.preemphasis.html
# I think the use of that really depends on the actual signal gathered, if the signal from the drones will be of low freq nature, i do not think it will help much to apply pre-emphasis
signal = librosa.effects.preemphasis(signal)

# the use of MFCC, although working in the current state, might not be the most optimal, as based on the warning from https://librosa.org/doc/main/generated/librosa.feature.mfcc.html
# "If multi-channel audio input y is provided, the MFCC calculation will depend on the peak loudness (in decibels) across all channels. The result may differ from independent MFCC calculation of each channel."
# So i presume it might be beneficiary to try to mfcc each channel separetly. TEST !

mfcc = librosa.feature.mfcc(
    y=signal,  # input signal
    sr=fs, # sampling rate
    n_mfcc=13,        # number of MFCC coefficients
    n_fft=1024,        # FFT size for win _length 100, 
    # df = 4000/128 = 31 Hz #shit, 4000/256 = 15 ... , 4000/516 = 7,75... 16000/1032 = 15Hz, 20000/1032 = 19,4 Hz
    hop_length=250,   # win_length / 2  = 50
    win_length=500,   # to get 25 ms = 0.025 * 4000 = 100, 0.025 * 16000 = 400, 0.025 * 20000= 500
    n_mels=40         # number of Mel filters
)

print("MFCC shape:", mfcc.shape)
# (number of mfc coefficients, number of frames)

 
# delta and delta-delta  
# https://librosa.org/doc/main/generated/librosa.feature.delta.html
# gives the local estimate of the derivative of the signal, so besically the velocity and acceleration of the mfcc, desribing the change over time, since
# the mfcc on its own only describes the spectral envelope at a given moment, but the sound varies over time.
mfcc_delta = librosa.feature.delta(mfcc)
mfcc_delta2 = librosa.feature.delta(mfcc, order=2)

# has a shape of (13, n_time_frames) per mfcc,mfcc_delta etc
# Stack features 
mfcc_features = np.vstack([mfcc, mfcc_delta, mfcc_delta2])
#print("TEST", mfcc_features)
# format
# (13, n...)
# (13, n...)
# (13, n...)
print("Final MFCC feature shape:", mfcc_features.shape)

plt.figure(figsize=(10, 4))
plt.specgram(
    signal,
    NFFT=1024,
    Fs=fs,
    noverlap=400,
    cmap="magma"
)
plt.xlabel("Time (s)")
plt.ylabel("Frequency (Hz)")
plt.title("Spectrogram (time–frequency view)")
plt.colorbar(label="Power")
plt.tight_layout()
plt.show()

mel_spec = librosa.feature.melspectrogram(
    y=signal,
    sr=fs,
    n_fft=1024,
    hop_length=250,
    win_length=500,
    n_mels=40,
    power=2
)


log_mel_spec = librosa.power_to_db(mel_spec, ref=np.max)

print(log_mel_spec)
# Accumulation of the values
# subject to change, depending on the ML !!!
logmel_stats = np.hstack([log_mel_spec.mean(axis=1), log_mel_spec.std(axis=1)])
mfcc_stats = np.hstack([mfcc.mean(axis=1),mfcc.std(axis=1)])
features = np.hstack([logmel_stats,mfcc_stats])

feature_names = (
    [f"mel_mean_{i}" for i in range(40)] +  # Avg log-energy in freq band over the whole recording, low value - low freq, high value high freq
    [f"mel_std_{i}"  for i in range(40)] + # How stable the energy is, low value steady mechanical sound, otherwise highly modulated sound such as voice
    [f"mfcc_mean_{i}" for i in range(13)] + # Spectral shape, different source different envelope
    [f"mfcc_std_{i}"  for i in range(13)] #stability of the spectral envelope, low vlaue - stable - drone / high value - unstable - speach
)
logmel_df  = pd.DataFrame([features], columns=feature_names)
logmel_df .to_csv("logmel_features.csv", index=False) 
print(logmel_df .columns)

plt.figure(figsize=(10, 4))
librosa.display.specshow(
    log_mel_spec,
    x_axis="time",
    y_axis="mel",
    sr=fs
)
plt.colorbar(label="dB")
plt.title("Log-Mel Spectrogram")
plt.tight_layout()
plt.show()