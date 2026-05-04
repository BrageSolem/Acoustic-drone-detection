from scipy.io.wavfile import write
from receivers.stm32_usb_receiver import STM32UsbReceiver
import numpy as np

class WavCreation:
    """
    Creates the wav sound file, using samples gathered with 
    stm32_usb_receiver
    
    Has one func called convert_into_wav(), that write the sound file into recordings folder. 
    """
    def __init__(self):
        pass

    def convert_into_wav(self, samples : np.ndarray, receive_time : STM32UsbReceiver, duration : STM32UsbReceiver):
        filename =f"debug_figures/04_05_drone_test_riktig_plassering/mic_recording{receive_time}.wav"
        effective_fs = len(samples[0])/ duration
        #print([len(ch) for ch in samples])
        #print(samples.shape)
        #print(samples.dtype)
        #print(effective_fs)
        audio = np.stack(samples, axis=1)
        audio = np.clip(audio, -32768, 32767).astype(np.int16)
        write(filename,int(round(effective_fs)), audio)