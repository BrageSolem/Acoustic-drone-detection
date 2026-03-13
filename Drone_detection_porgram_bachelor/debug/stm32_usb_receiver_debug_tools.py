from receivers.stm32_usb_receiver import STM32UsbReceiver 
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class ReceiverDebug:
    def __init__(self, receiver :STM32UsbReceiver):
        self.receiver = receiver

    def debug_export (self, plot_debug = False):
        samples_dict = {}

        if not self.receiver.samples:
            raise RuntimeError("No samples available. Run receiver.record() first.")

        for channel in range(self.receiver.channels):
            name = f"mic{channel+1}"
            samples_dict[name] = np.array(self.receiver.samples[channel])

        if plot_debug:

            plt.figure()

            for channel in range(self.receiver.channels):
                
                name = f"mic{channel+1}"

                plt.subplot(self.receiver.channels,1,channel + 1)
                plt.plot(samples_dict[name][:2000])
                plt.title(f"Mic {channel + 1}")
                print(f"Total samples mic{channel+1}: {len(samples_dict[name])}")
            
            plt.tight_layout()
            plt.savefig(f'debug_figures/Debug_receiver_data{int(self.receiver.start_time)}.png')

        df = pd.DataFrame(samples_dict)

        df.to_csv("debug_csv_files/mic_recording_usb.csv", index=False) 