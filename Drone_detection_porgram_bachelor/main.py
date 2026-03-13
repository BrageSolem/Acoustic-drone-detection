from receivers.stm32_usb_receiver import STM32UsbReceiver
from dataset_tools.create_wav_file import WavCreation
from features.mfcc_extraction import  MFCCExtractor
from features.feature_exporter import FeatureExporter
from features.feature_visualizer import FeatureVisualizer
from debug.stm32_usb_receiver_debug_tools import ReceiverDebug
from gcc.gcc_processor import GCCProcessor

"WORK IN PROGRESS !!"

debug_mfcc = False
debug_stm32 = False
Run = True

receiver = STM32UsbReceiver()
receiver_debug = ReceiverDebug(receiver=receiver)

wav_conversion = WavCreation()

mfcc_extractor = MFCCExtractor()
mfcc_exporter = FeatureExporter(extractor=mfcc_extractor)
mfcc_visualizer = FeatureVisualizer(extractor=mfcc_extractor)

gcc_processor = GCCProcessor(duration = receiver.duration_sec, channels = receiver.channels)

receiver.open_port()
# add history buffer for 3 wav files back in time ?


while Run:
    samples = receiver.record() # record the data sent by the stm32
    
    wav_conversion.convert_into_wav(samples) # convert the samples into a wav format
    
    mfcc_extractor.extract_features(audio_file="recordings/mic_recording.wav") # extract features such as mfcc, delta, delta2, log_mel_spec etc
    mfcc_exporter.df_features() # export the features into a df
    gcc_processor.process(samples, mfcc_extractor.fs)
    gcc_processor.


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