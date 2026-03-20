from receivers.stm32_usb_receiver import STM32UsbReceiver
from dataset_tools.create_wav_file import WavCreation
from features.mfcc_extraction import  MFCCExtractor
from features.feature_exporter import FeatureExporter
from features.feature_visualizer import FeatureVisualizer
from debug.stm32_usb_receiver_debug_tools import ReceiverDebug
from gcc.gcc_processor import GCCProcessor
from gcc.doa_estimator import DOAEstimator

import numpy as np

"WORK IN PROGRESS !!"

debug_mfcc = False
debug_stm32 = False
Run = True

# Position vector for 4 microphones
# They are planned to be placed on the same height, and around 10 cm from the center of the tower
p_vector = np.array([
    [-0.10,  0,     0.10,  0   ],  # x: mic1 left, mic3 right
    [ 0,    -0.10,  0,     0.10],  # y: mic2 bottom, mic4 top
    [ 0,     0,     0,     0   ]]) # z


receiver = STM32UsbReceiver()

wav_conversion = WavCreation()

mfcc_extractor = MFCCExtractor()
mfcc_exporter = FeatureExporter(extractor=mfcc_extractor)
mfcc_visualizer = FeatureVisualizer(extractor=mfcc_extractor)

gcc_processor = GCCProcessor()
doa_estimator = DOAEstimator(p_vector, gcc_processor)

#debug 
receiver_debug = ReceiverDebug(receiver=receiver)

receiver.open_port()
# add history buffer for 3 wav files back in time ?


while Run:
    samples = receiver.record() # record the data sent by the stm32
    
    wav_conversion.convert_into_wav(samples) # convert the samples into a wav format
    
    # features 
    features = mfcc_extractor.extract_features(audio_file="recordings/mic_recording.wav") # extract features such as mfcc, delta, delta2, log_mel_spec etc
    mfcc_exporter.df_features() # export the features into a df
    
    # Direction
    gcc_array = gcc_processor.process_signal(samples, mfcc_extractor.fs)
    doa_estimator.set_gcc_array(gcc_array)
    doa_estimator.estimate_DOA()


    """
    #Simple example of rule based drone detection

    mfcc_std = features[-13:] # last mfcc std
    mfcc_std_mean = np.mean(mfcc_std)

    is_drone = mfcc_std_mean < threshold    
    

    if i_drone:
        angle = doa_estimator.estimated_azimuth_deg
    else:
        ignore

    # then send the angle to the PID

    """










    if debug_mfcc:
        mfcc_visualizer.plot_signal_time()
        mfcc_visualizer.spectrogram()
        mfcc_visualizer.mel_spectrogram()
        Run = False

    if debug_stm32:
        receiver_debug.debug_export(plot_debug=True)
        Run = False 
    
#receiver.close_port()
#receiver.debug_export(plot_debug = True)