import numpy as np
from scipy.fft import fft
from scipy import signal


class GCCProcessor:
    def __init__(self, frame_ms = 32):
        self.frame_ms = frame_ms


    def _frame_samples(self,samples: np.ndarray, samples_per_frame):

        # samples shape (n_mic, values)
        #   mic1 : s1, s2, s3 , ...
        #   mic2 : s1, s2, s3 , ...
        #   mic3 : s1, s2, s3 , ...
        #   mic4 : s1, s2, s3 , ...

        frames = []
        samples_n = samples.shape[1]
        hop = samples_per_frame//2
        for frame_n in range(int(samples_n/hop)):
            start = frame_n * hop
            end = start + samples_per_frame

            if end <= samples_n:
                frames.append(samples[:, start:end])
        return np.array(frames)
    
        # frames shape (n_frame, n_mic, values)
        # frame[0]
        #   mic1 : s1, s2, s3 , ...
        #   mic2 : s1, s2, s3 , ...
        #   mic3 : s1, s2, s3 , ...
        #   mic4 : s1, s2, s3 , ...
        # 
        # frame[1] .... 
    
    def _window_each_frame(frames, samples_per_frame):
        hanning = np.hanning(int(samples_per_frame))
        windowed_frames = frames * hanning
        return windowed_frames


    def process(self, samples: np.ndarray, fs): #samples has to be np.array to use .shape
        duration = samples.shape[1]/fs 
        channels = samples.shape[0]
        samples_per_frame = int(fs * self.frame_ms / 1000)

        frames = self._frame_samples(samples, samples_per_frame=)
        windowed_frames = self._window_each_frame(frames, samples_per_frame)


    

        
