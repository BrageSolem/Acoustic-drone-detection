import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.fft import fft, ifft, fftshift
import matplotlib.pyplot as plt
from scipy import signal
#from scipy.io import wavfile

df = pd.read_csv("mic_recording_usb.csv")
#fs, _ = wavfile.read("mic_recording_04_02_1.wav")
mic1, mic2, mic3, mic4 = [df[col].to_numpy() for col in df.columns]
#mic1, mic4 = [df[col].to_numpy() for col in df.columns]
"""
1.Frame the signals - DONE

2. Window each frame  - DONE

3. FFT each frame - DONE

4. Compute cross power spectrum per mic pair per frame - DONE

5. Apply PHATβ - DONE

6. Inverse FFT - DONE

7. Find peak lag - DONE

8. Repeat for all mic pairs - DONE

9. Test with greater spacing of the mics

"""
# 1 framing of the signals to 32 ms based on the pre-filter paper 
#duration = 4
#fs_pcm = 3_000_000/128 # theoretical signal freq from the mics/ decimation factor
fs_pcm = 23437.5#len(mic1)/duration
#fs = int(len(mic1) / duration) # samples per second, feil, droper frames
frame_ms = 32 # ms
samples_per_frame = int(fs_pcm * frame_ms / 1000) # samples per frame of 32 ms

#n_frames = len(mic1) // samples_per_frame
# THE NUMBER OF SAMPLES PER FRAME CHANGES WITH FS!!!! WITH THE FS OF 23437, the value will be around 750 samples per frame
# samples_per_frame * frame_number * 0.5 : number from the left side + samples_per_frame 
# 284 * 0 * 0.5 : 0 + 284 ----- [0 : 284]  - each frame must have 284 samples, to represent 32 ms, but acoriding to this paper: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7602964 , there is an overlap at the centre of each frame, to not lose the samples i presume 
# 284 * 1 * 0.5 : 142 + 284 --- [142 : 426]
# 284 * 2 * 0.5 : 284 + 284 --- [284 : 568]
mic1_frames_td = []
mic2_frames_td = []
mic3_frames_td = []
mic4_frames_td = []

#for frame_n in range(int(len(mic1)/samples_per_frame)):
#    mic1_frames.append(mic1[int(samples_per_frame * frame_n * 0.5): samples_per_frame + int(samples_per_frame * frame_n * 0.5)])
def frame_mic (mic, samples_per_frame, mic_frame_list):
    hop = samples_per_frame // 2 # same as samples_per_frame * 0.5 but guarantes the hop_size to be an int
    # hop = 284 // 2 = 142 jump of 142 samples
    for frame_n in range(int(len(mic)/hop)): # uses hop to produce the number of frames wich overlaps with the previous frames
        # OLDY NOT GOODY : for frame_n in range(int(len(mic)/sample_per_frame)):
        # could be useful, if the frames would not be expected to overlap
        start = frame_n * hop
        # 1 it : 0 * 142 = 0
        # 2 it : 1 * 142 = 142 
        end = start + samples_per_frame
        # 1 it : (0 * 142) + 284 = 284
        # 2 it : (1 * 142) + 284 = 426

        if end <= len(mic): # discard the last frame that has less samples then samples_per_frame. if len is 88_000 and end is 88_142 then the last frame consisting of the last known 142 samples and 142 NON existing values
            mic_frame_list.append(mic[start : end])
            # 1 it : mic[0 : 284]
            # 2 it : mic[142 : 426] 50% gamle og 50% nye verdier

def window_each_frame(mic_frame_list, samples_per_frame):
    hanning = np.hanning(int(samples_per_frame)) # has to mach the hop
    for i in range(len(mic_frame_list)):
        mic_frame_list[i] = mic_frame_list[i] *  hanning
    mic_frame_list = np.array(mic_frame_list)
    return mic_frame_list

def fft_each_frame (mic_frame_list):
    mic_frame_list_fd = fft(mic_frame_list, axis=-1)
    # changed the input from list to an np.array #muchBetterNow
    #print(mic_frame_list_fd.shape) // # i = N_frames
    #for n,frame in enumerate(mic_frame_list): # had to use that to make sure the frames are not mixed up during the fft process
    #    mic_frame_list_fd[n, :] = fft(frame) # row n all freq
        #mic_frame_list_fd.append(fft(frame))
    return mic_frame_list_fd

# cross power spectrum 
# new_list = [expression for item in iterable if condition]

def cross_power_spectrum (mic1_fft, mic2_fft, mic3_fft, mic4_fft):  # has to apply the values from freq domain
    """
    Based on this article https://ieeexplore.ieee.org/abstract/document/7602964, the form of cross correlation at lag k is
    G(k) = X1(k) * X2^*(k) where each X2 is the conjugate of it self.
    
    """
    
    cps12 = mic1_fft * mic2_fft.conj()
    cps13 = mic1_fft * mic3_fft.conj()
    cps14 = mic1_fft * mic4_fft.conj()
    cps23 = mic2_fft * mic3_fft.conj()
    cps24 = mic2_fft * mic4_fft.conj()
    cps34 = mic3_fft * mic4_fft.conj()
    
    return cps12,cps13, cps14, cps23,cps24,cps34
# mic 1      
# frame 0 : [bin0 = -220.72186545   -0.j, bin1 = 1327.80626346-1043.86367388j, bin2 = -2205.87856392 -469.19216121j ]
# f 1 : [299.7554295    -0.j,  -489.20599467  +56.94168586j, 227.67937369  +11.68397539j ]

# mic 2 
# f 0 : [-546.93490178   -0.j , 1418.52916371 -918.86202563j,-2110.81371543 -405.97796896j ]
# f 1 : [487.9303065    -0.j, -915.34764659 -262.7317206j, 1017.99715204 +387.73763226j]

# conjugate of mic 2 
# f 0 : [-546.93490178   +0.j , 1418.52916371 +918.86202563j,-2110.81371543 +405.97796896j ]
# f 1 : [487.9303065    0.j, -915.34764659 262.7317206j, 1017.99715204 -387.73763226j]

# G_{12}^(n)(k) = X_1^(n)(k) * X_2^(n*)(k) 
# So basically the element wise multiplication - so each bin multyiplys by the corresponding elemeent
# phase(k) = atan(Im(k),Re(k))
# np.atan(np.imag(), np.real())
# Bin 0 : -220.72186545   -0.j, 1327.80626346-1043.86367388j * -546.93490178   +0.j = something

"""
# Makes all of the reqs to contribute equaly, despite some of the frq not having much energy/utility
def phat(cps_mics): # has to be an array
    epsilon = 1e-13 # just for safety, in case the absolute value of cps would be zero
    phats = []
    for cps in cps_mics:
        phats.append(cps / (np.abs(cps) + epsilon))
    #phat_res = cps_mics/(np.abs(cps_mics) + epsilon) # normalizes -  removes the magnitude info
    
    return phats
"""
"""
plt.figure()
plt.plot(np.arange(len(mic1)), mic1, label = "Raw mic signal")
plt.legend()
#print(mic1)
"""
# will have to change the freq of the low and high based on the drone testing
def phat_weighted(cps_mics, fs, n_samples_frame, freq_low = 200, freq_high = 6000, beta=0.5, eps = 1e-13, use_adaptive_beta = False):
    """
    The function is used to filter the correlation values from the frequency bins present outside of the desired band frequency.
    And to reduce the impact of low energy freq bins present in the desired band.    
    """
    freqs = np.fft.fftfreq(n_samples_frame, d = 1/fs)
    band = (np.abs(freqs) >= freq_low ) & (np.abs(freqs) <= freq_high) # reduce the influence of the low and high freq noise, has to be adjusted based on the sound from the drones
    mag = np.abs(cps_mics) + eps # eps added to guarantee that there will be no division by zero, 
    # uses beta to reduce the influence from the weak bins, explain in more detail
    # based on the article a standard gcc phat is as follows:
    # Gx1x2 * e^(j2pifT) / |Gx1x2 * e^(j2pifT)| or a + bj / | a+ bj| 
    # but based on the second article, it may be beneficiary to introduce change of the value of B which in classical PHAT is equal 1
    # while for the standard GCC the value of Beta is 0.
    # So based on the article, the value of B close to 1 performs best at High SNR, while at low SNR value lesser then 1 performs better.
    # estimation of value of B can be conducted by finding the SNR of the signal by dividing the mean of the signal, by the median of the signal
    # in this way the mean represents mostly the signal, as mean is much more ceceptable to the extreme values, which are expected to be representative 
    # of the actual signal, and the median is to represent the noise present in the signal, since the noise is expected to be 
    # uncorelated with the desired sound signal and random in nature

    if use_adaptive_beta:
        signal_p = np.mean(mag)
        noise_p = np.median(mag) + eps
        snr_est = signal_p / noise_p

        beta = snr_est / (snr_est + 1) #taken from eq.6 On pre-filtering strategies for the GCC-PHAT algorithm

    weighted_cps = cps_mics / (mag**beta)  # https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5670137 and https://ieeexplore.ieee.org/abstract/document/7602964 
    
    weighted_cps *= band # the bins outside of the band will be zeroed

    gcc = np.fft.ifft(weighted_cps, axis = -1) # eq.18, page 2 analysis of the gcc-phat..
    gcc = np.fft.fftshift(gcc, axes = -1) # places the 0 bin at the center.  

    return np.real(gcc)


frame_mic(mic1, samples_per_frame, mic1_frames_td)
frame_mic(mic2, samples_per_frame, mic2_frames_td)
frame_mic(mic3, samples_per_frame, mic3_frames_td)
frame_mic(mic4, samples_per_frame, mic4_frames_td)

#print(len(mic1_frames_td[0][:]))
"""
plt.figure()
plt.plot(np.arange(len(mic1_frames_td[0][:])), mic1_frames_td[0][:], label = "THe first frame of the  raw signal")
plt.legend()
"""
mic1_frames_td = window_each_frame(mic1_frames_td, samples_per_frame)
mic2_frames_td = window_each_frame(mic2_frames_td, samples_per_frame)
mic3_frames_td = window_each_frame(mic3_frames_td, samples_per_frame)
mic4_frames_td = window_each_frame(mic4_frames_td, samples_per_frame)
"""
plt.figure()
plt.plot(np.arange(len(mic1_frames_td[0][:])), mic1_frames_td[0][:], label = "THe first frame of the  windowed signal")
plt.legend()
"""
#print(mic1_frames_td)
#mic1_frames_td = np.array(mic1_frames_td)
#print(type(mic1_frames_td), mic1_frames_td.shape)
#print("before fft", mic1_frames[0])

mic1_frames_fd = fft_each_frame(mic1_frames_td)
mic2_frames_fd = fft_each_frame(mic2_frames_td)
mic3_frames_fd = fft_each_frame(mic3_frames_td)
mic4_frames_fd = fft_each_frame(mic4_frames_td)
"""
plt.figure()
plt.plot(np.arange(len(mic1_frames_fd[0][:])), np.abs(mic1_frames_fd[0][:]), label = "THe first frame of the  windowed signal in F - domain")
plt.legend()
"""
#print("after fft", mic1_frames[0])
#print(int(len(mic1)/samples_per_frame)) # 0% overlap
#print(int(len(mic1)/(samples_per_frame//2))) # 50% overlap

#print("Test 1", mic1_frames_td[0])
#print("Test 2" ,mic1_frames_fd[0])

#print(mic1_frames_fd[0], type(mic1_frames_fd))
#print(type(mic1_frames_fd),mic1_frames_fd.shape)
#print(mic1_frames_td, type(mic1_frames_td))

### DO THAT FOR ALL OF THE MICROPHONES

cps12,cps13, cps14, cps23,cps24,cps34 = cross_power_spectrum(mic1_frames_fd, mic2_frames_fd, mic3_frames_fd, mic4_frames_fd)
cps_arr = [cps12,cps13, cps14, cps23,cps24,cps34]
#print(cps_mic1_mic2)
#print(mic2_frames_fd)
gcc_list = []

for cps in cps_arr:
    gcc = phat_weighted(cps, fs_pcm, samples_per_frame, use_adaptive_beta= True)
    gcc_list.append(gcc)

"""
plt.figure()
plt.plot(np.arange(len(cps_mic1_mic4[0][:])), np.abs(cps_mic1_mic4[0][:]), label = "CPS first frame")
plt.legend()
"""

#phats = phat(cps_arr) # phase difference between the signals from the mics
#print(phat_mic1_mic2)
#inversed_array =  [ifft(phat) for phat in phats]
#inversed_centered_array = [fftshift(inv_phat, axes = -1) for inv_phat in inversed_array]
#inversed_phat_mic1_mic4 = ifft(phat_mic1_mic4, axis=-1)
#inversed_phat_centered_mic1_mic4 = fftshift(inversed_phat_mic1_mic4, axes = -1)


def delta_newton_approx(y, k): # to some degree based on https://pages.hmc.edu/ruye/MachineLearning/lectures/ch3/node4.html
    """
    To find the highest correlation value of the lag present in each frame, the func argmax() was used in the main loop of the program.
    It may turn out to be a bit problematic as the signals measured are of continues nature, but the signals used for the correlation purposes
    are of discrete nature, making them dependent on the number of samples in each frame.

    The argmax func returns the index value of the  (bin) that has the highest value, but since the real signal worked on is continous, it is possible
    that the sample is not representing the exact, highest value present in the frame of the continous signal, but a value that is close to it.
    
    That matters because of how long time intervals each sample (lag) represents. 
    Ts = 1/fs
    Ts = 1/ (3_000_000 / 128) # fs value comes from PDM2PCM which decimates the signal from mics with a factor of 128 (had to choose the highest possible in the stm32 lib, due to the limitations of the UART)
    Ts = ca 42.7 micro s

    so each lag represents around 42.7 micro seconds.
    In the scenario where the peak is placed halfway between two samples, the argmax() will have to choose one of the samples causing the time error of
    Ts / 2 = 42.7 / 2 = 21.35 micro seconds
         top
    |-----'------|-----------| 
    k-1          k          k+1

    which can froce the program to detect the drone presence from the wrong diraction.

    It is assumed that the peak is locally parbolic, so it is possible that the value hold by the sample is before or after the true maximum, not accuratly representing the actual delay between the signals
    hence, true_max_peak = estimated_max_peak + delta .

    """

    if k <= 0 or k >= len(y) - 1:
        return 0.0
    # has to find the stationary point, or a point where f'(x) = 0 

    # newton - raphson 
    # x1 = x0 - f'(x)/f''(x)
    # delta = x1 - x0 
    # delta = - f'(x)/f''(x)

    y0 = y[k - 1] 
    y1 = y[k]
    y2 = y[k + 1]

    # finite -diff aprrox of the 1st derivative 
    # (y[k+1] - y[k-1]) / (2*h)
    b = 0.5 * (y2 - y0) # centered approx of f'(x) when h = 1

    # finite-diff approx of the 2nd derivative 
    # y[k-1] - 2y[k] + u[k+1] / h** 2 
    # since it is the index space, h = 1
    # y[k-1] - 2y[k] + y[k+1]

    a = y2 + y0 - 2*y1
    if np.abs(a) < 1e-15: # in case a is (almost) 0, to prevent division by 0
        return 0.0
    
    delta = -b / a
    return float(delta)


N_frames, K_bins = gcc_list[0].shape
lag_center = K_bins//2
#print(lag_center)
corr_mags = [np.abs(gcc) for gcc in gcc_list]
#corr_mag = np.abs(inversed_phat_centered_mic1_mic4)

"""
plt.figure()
plt.plot(np.arange(K_bins) - lag_center, corr_mag[0,:], label = "corr_mag whole lag spectrum for the first frame") # y - normalized similarity score, between the signals
plt.legend()
plt.show()
"""

speed_of_sound = 343 #spead of sound
#distance_mics = 0.1 #distance 14 cm

"""
peak_lags = np.zeros(N_frames, dtype = int)
for n in range(N_frames):
    local_peak = np.argmax(corr_mag[n, lag_min:lag_max]) # all frames, bins from min to max, but without max, hence 1 added beforehand
    peak_lags[n] = (lag_min + local_peak) - lag_center
time_delays =  peak_lags/ fs
aggregated_time_delay = np.median(time_delays) # single frame can be quite noisy, so takes the median of the time delays, the reason to use median instead of mean, is the fact that median is less sensitive to outliers
"""

peak_lags = np.zeros(N_frames, dtype=np.float64) # holds the estimated lag for each frame
confidence_score = np.zeros(N_frames, dtype=np.float64) 
pair_names = ["1–2", "1–3", "1–4", "2–3", "2–4", "3–4"] # make into a dict?
pair_dists = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1 ] # 0.10 m for 10 cm 

for pair_indx in range(len(corr_mags)):
    peak_lags = np.zeros(N_frames)
    confidence_score = np.zeros(N_frames)
    
    distance_mics = pair_dists[pair_indx]
    t_delay_max = distance_mics / speed_of_sound
    max_lag_samples = int(np.ceil((t_delay_max) * fs_pcm))  # max delay in samples, lag = delay, had to use ceil to make sure none valid delays are cut, but can one impossible delay. with round(9.01) = 9, lost some info
    lag_min = max(0,lag_center - max_lag_samples)
    lag_max = min(K_bins, lag_center + max_lag_samples + 1)

    for n in range(N_frames):
        # by applying segments, only the phisically possible lags given microphone spacing will be looked at
        a_frame = corr_mags[pair_indx][n, lag_min:lag_max]  # only physically valid lags, f.ex frame 0 - lags from min to max

    # integer peak inside the segment
    # returns the index of the value that is biggest for that segment, and ONLY the segment/current frame
    # so the lag corresponding to the strongest correlation indicates how many samples one entire frame must be shifted relative to the other for best alignment
        peak_this_frame = int(np.argmax(a_frame)) 

    # sub-sample correction using parabola (works on the local segment)
        delta = delta_newton_approx(a_frame, peak_this_frame) # CONTINUE FROM HERE !

    # convert to lag in samples (relative to lag_center)
    # since the lag peak might be a non int, k_local(int) + delta(decimal) = decimal - this imrpoves the acccuracy of the lag estimation
    # have to add the lag_min to convert the index into the global array, and since lag = 0 is at lag_center, by subtracting it, 
    # we get the pos/negative value representing which mic has received the signal first
        est_lag_samples_for_n = (lag_min + (peak_this_frame + delta)) - lag_center # estimated TDOA in samples for frame n
        peak_lags[n] = est_lag_samples_for_n 

    # simple confidence: peak height vs median energy in the segment
        confidence_score[n] = a_frame[peak_this_frame] / (np.median(a_frame) + 1e-15) # has to use median, as mean is to easly influenced by the outliers
    
    time_delays = peak_lags / fs_pcm # all of the time delays in s
#print(time_delays)
    keep_frac = 0.2
    k = int(np.ceil(keep_frac * N_frames))
    idx = np.argsort(confidence_score)[-k:]

    aggregated_time_delay = np.median(time_delays[idx])
# Optional: only keep frames with a clear peak (tune threshold 2-6)
    #good = confidence_score > 3.0
    #if np.any(good):
    #    aggregated_time_delay = np.median(time_delays[good]) # only the "good" reading will be checked
    #else:
    #    aggregated_time_delay = np.median(time_delays)

    print("------------------------")
    print(f"Pair: {pair_names[pair_indx]}")
    print("aggregated_time_delay:", aggregated_time_delay) # -... value the 1 mic got the signal first 
    print("fs:", fs_pcm)
# one sample should have around 42 micro seconds since Ts = 1/fs
    print("max_lag_samples:", max_lag_samples)
    #print("good frames:", np.sum(good), "/", len(good))
    print("confidence median:", np.median(confidence_score), "confidence max:", np.max(confidence_score))
    print("lag in samples: ", aggregated_time_delay * fs_pcm)

plt.figure()
plt.plot(np.arange(lag_min, lag_max) - lag_center, corr_mags[2][n, lag_min:lag_max]) # the last frame, n still exists bro
plt.xlabel("Lag [samples]")
plt.ylabel("GCC-PHAT")
plt.title("Zoomed GCC-PHAT (physically valid lags)")
plt.grid()
plt.show()

print("max abs:", np.max(np.abs(mic1)), np.max(np.abs(mic2)), np.max(np.abs(mic3)), np.max(np.abs(mic4)))



"""
print(aggregated_time_delay)
print("fs:", fs)
print("max_lag_samples:", max_lag_samples)
print("unique peak lags:", np.unique(peak_lags, return_counts=True))

n = 0
plt.plot(corr_mag[n, :])
plt.show()
"""