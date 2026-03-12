from receivers.STM32UsbReceiver import STM32UsbReceiver

receiver = STM32UsbReceiver()

receiver.open_port()
mic1, mic2, mic3, mic4 = receiver.record()
receiver.debug_export(plot_debug = True)
receiver.close_port()