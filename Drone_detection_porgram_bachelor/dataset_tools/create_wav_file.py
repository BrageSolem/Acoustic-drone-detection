from scipy.io.wavfile import write
import numpy as np

class WavCreation:
    """
    Creates the wav sound file, using samples gathered with 
    stm32_usb_receiver
    
    Has one func called convert_into_wav(), that write the sound file into recordings folder. 
    """
    def __init__(self, duration = 1):
        self.duration = duration

    def convert_into_wav(self,samples : np.ndarray,filename ="recordings/mic_recording.wav" ):
        effective_fs = len(samples[0])/ self.duration
        stereo = np.stack(samples, axis=1)
        write(filename,int(round(effective_fs)), stereo)