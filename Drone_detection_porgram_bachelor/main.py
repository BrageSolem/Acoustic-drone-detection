from receivers.stm32_usb_receiver import STM32UsbReceiver
from dataset_tools.create_wav_file import WavCreation
from features.mfcc_extraction import  MFCCExtractor
from features.feature_exporter import FeatureExporter
from features.feature_visualizer import FeatureVisualizer
from debug.stm32_usb_receiver_debug_tools import ReceiverDebug
from gcc.gcc_processor import GCCProcessor
from gcc.doa_estimator import DOAEstimator
from gcc.doa_visualizer import DOAVisualizer
from ml.accumulativemodel import AccumulativeModel
from ml.cnnmodel import CnnModel
from utilis.drone_client import DroneSocketClient

import numpy as np

"WORK IN PROGRESS !!"

debug_mfcc = False
debug_stm32 = False
Run = True

# Position vector for 4 microphones
# They are planned to be placed on the same height, and around 10 cm from the center of the tower
"""
p_vector = np.array([
    [ -0.055,  -0.03, -0.07,  0.105],  # x: mic3 left, mic4 right
    [-0.075,  0.095,  0.01,  0.005],  # y: mic1 bottom, mic2 top
    [ 0.0,  0.0,  0.0,  0.0]   # z
])
"""

p_vector = np.array([
    [ 0.0,  0.0, -0.1,  0.1],  # x: mic3 left, mic4 right
    [-0.1,  0.1,  0.0,  0.0],  # y: mic1 bottom, mic2 top
    [ 0.0,  0.0,  0.0,  0.0]   # z
])


receiver = STM32UsbReceiver()


wav_conversion = WavCreation()

mfcc_extractor = MFCCExtractor()
mfcc_exporter = FeatureExporter(extractor=mfcc_extractor)
mfcc_visualizer = FeatureVisualizer(extractor=mfcc_extractor)

gcc_processor = GCCProcessor()
doa_estimator = DOAEstimator(p_vector, gcc_processor)
doa_visualizer = DOAVisualizer()

accumulative_model = AccumulativeModel()
cnn_model = CnnModel()

drone_client = DroneSocketClient(server_url='http://localhost:3001',on_init=on_init, on_update=on_update)

#debug
receiver_debug = ReceiverDebug(receiver=receiver)

receiver.open_port()
# add history buffer for 3 wav files back in time ?
i = 0
try:
    drone_client.connect()
except Exception as e:
    print('connection error:', e)

while Run:
    i+=1
    samples = receiver.record() # record the data sent by the stm32
    # test DC offset 
    samples= samples.astype(np.float32)
    samples = samples - np.mean(samples)

    wav_conversion.convert_into_wav(samples, receive_time=receiver.start_time, duration= receiver.duration_sec) # convert the samples into a wav format
    receiver_debug.debug_export(plot_debug=True)

    # features 
    features = mfcc_extractor.extract_features(audio_file=f"debug_figures/04_05_drone_test_riktig_plassering/mic_recording{receiver.start_time}.wav") # extract features such as mfcc, delta, delta2, log_mel_spec etc
    mfcc_exporter.df_features() # export the features into a df
    # adds a lot of overhead, comment out during live testing
    #mfcc_visualizer.mel_spectrogram(receiver.start_time)
    #mfcc_visualizer.spectrogram(receiver.start_time)
    
    # Sound classification
    cnn_predict = cnn_model.cnn_predict(f"debug_figures/04_05_drone_test_riktig_plassering/mic_recording{receiver.start_time}.wav") #Fix, add permanent non debug location
    accumulative_predict = accumulative_model.acum_predict(features)
    
    if cnn_predict == "Drone" and accumulative_predict == "Drone":
        # Direction
        gcc_array = gcc_processor.process_signal(samples, mfcc_extractor.fs)
        doa_estimator.set_gcc_array(gcc_array)
        doa_estimator.estimate_DOA()
        print(f"Azimuth this iteration is: {doa_estimator.estimated_azimuth_deg}")
        print(f"Iteration: {i}")
        drone_client.emit_drone("drone")
        drone_client.emit_degree(doa_estimator.estimated_azimuth_deg)
        # adds a lot of overhead, comment out during live testing
        #doa_visualizer.visualize_doa(doa_estimator.estimated_azimuth_deg, doa_estimator.estimated_azimuth_rad, #receiver.start_time)
        
    if i == 45: # allows for 45 iterations, before the program terminates
        Run = False   
    # debug 
    print(f"CNN prediction: {cnn_predict}")
    print(f"Accumulative prediction: {accumulative_predict}")
    
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
