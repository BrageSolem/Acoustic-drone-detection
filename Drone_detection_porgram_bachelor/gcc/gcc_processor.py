import numpy as np
from scipy.fft import fft
from scipy import signal


class GCCProcessor:
    def __init__(self, frame_ms = 32):
        self.frame_ms = frame_ms
        self.mic_pairs = None


    def _frame_samples(self,samples: np.ndarray, samples_per_frame):

        # samples shape (n_mic, values)
        #   mic1 : s1, s2, s3 , ...
        #   mic2 : s1, s2, s3 , ...
        #   mic3 : s1, s2, s3 , ...
        #   mic4 : s1, s2, s3 , ...
        if samples is not None:
            frames = []
            samples_n = samples.shape[1]
            hop = samples_per_frame//2
            for frame_n in range(int(samples_n/hop)):
                start = frame_n * hop
                end = start + samples_per_frame

                if end <= samples_n:
                    frames.append(samples[:, start:end])
            return np.array(frames)
        else:
            raise RuntimeError("No samples present, gather them again")
        # frames shape (n_frame, n_mic, values)
        # frame[0]
        #   mic1 : s1, s2, s3 , ...
        #   mic2 : s1, s2, s3 , ...
        #   mic3 : s1, s2, s3 , ...
        #   mic4 : s1, s2, s3 , ...
        # 
        # frame[1] .... 
    
    def _window_each_frame(self,frames: np.ndarray):
        hanning = np.hanning(int(frames.shape[-1]))
        windowed_frames = frames * hanning
        return windowed_frames
    
    def _fft_each_frame(self,frames):
        frames_frq_domain = fft(frames, axis=-1)
        return frames_frq_domain
    
    def _generate_mic_pairs(self, n_mics): # is quite needed in the tdoa and doa
        return [(i,j) for i in range(n_mics) for j in range(i+1, n_mics)]

    def _cross_power_spectrum(self,frames_fft:np.ndarray):
        # try to vectorize?
        # use list comprahension?
        cps = []
        n_frames = frames_fft.shape[0]
        n_mics = frames_fft.shape[1]

        if self.mic_pairs is None:
            self.mic_pairs = self._generate_mic_pairs(n_mics)
        
        mic_pairs = self.mic_pairs

        # for each frame multi mic_i with the conjugate of mic_j

        for frame in range(n_frames): 
            frame_cps = [] # cps of one frame
            for i,j in mic_pairs:
                cps_ij = frames_fft[frame, i, :] *frames_fft[frame,j,:].conj()
                frame_cps.append(cps_ij)
            cps.append(frame_cps)

        # frame_cps shape (frame, mic pair, cps)
        return np.array(cps)
        
    def _p_hat_weighted_gcc(self,cps_array :np.ndarray, fs, n_samples_per_frame, freq_low = 200, freq_high = 6000, beta = 0.5, eps = 1e-13, use_adaptive_beta = False ):
        freq = np.fft.fftfreq(n_samples_per_frame, d = 1/fs) # array of freq for each fft bin
        band = (freq_low <= np.abs(freq)) & (np.abs(freq) <= freq_high) # boolean mask to remove unwanted freq
        mag = np.abs(cps_array) + eps
        
        weighted_cps = cps_array/(mag**beta)
        weighted_cps *= band 
        gcc_array = np.fft.ifft(weighted_cps, axis =-1)
        gcc_array = np.fft.fftshift(gcc_array, axes = -1)

        return np.real(gcc_array)

    
    def process_signal(self, samples: np.ndarray, fs): #samples has to be np.array to use .shape
        samples_per_frame = int(fs * self.frame_ms / 1000)
        frames = self._frame_samples(samples, samples_per_frame)
        windowed_frames = self._window_each_frame(frames, samples_per_frame)
        frames_fft = self._fft_each_frame(windowed_frames)
        cps_array = self._cross_power_spectrum(frames_fft)
        gcc_array = self._p_hat_weighted_gcc(cps_array, fs, samples_per_frame)
        return gcc_array
    

        
