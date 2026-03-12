from receivers.stm32_usb_receiver import STM32UsbReceiver
from data_set_tools.create_wav_file import WavCreation

receiver = STM32UsbReceiver()
wav_conversion = WavCreation()

receiver.open_port()
# add history buffer for 3 wav files back in time ?

while True:
    samples = receiver.record()
    wav_conversion.convert_into_wav(samples)
    
#receiver.close_port()
#receiver.debug_export(plot_debug = True)