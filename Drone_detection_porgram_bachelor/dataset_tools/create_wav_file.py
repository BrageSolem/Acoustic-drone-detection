from scipy.io.wavfile import write
import numpy as np

class WavCreation:
    def __init__(self, duration = 1):
        self.duration = duration

    def convert_into_wav(self,samples,filename ="recordings/mic_recording.wav" ):
        effective_fs = len(samples[0])/ self.duration
        stereo = np.stack(samples, axis=1)
        write(filename,int(round(effective_fs)), stereo)