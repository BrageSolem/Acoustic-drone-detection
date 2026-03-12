




"WORK IN PROGRESS!"





def spectrogram(self):
        if self.signal is not None:
            plt.figure(figsize=(10, 4))
            plt.specgram(self.signal,
                         self.n_fft, 
                         self.fs, 
                         noverlap=400,
                         cmap = "magma")
                
            plt.xlabel("Time (s)")
            plt.ylabel("Frequency (Hz)")
            plt.title("Spectrogram (time–frequency view)")
            plt.colorbar(label="Power")
            plt.tight_layout()
            plt.show()
        else:
            print("The signal is empty! Run mfcc_after_preemphasis()")

def mel_spectogram(self):
        if self.log_mel_spec is not None:
            plt.figure(figsize=(10, 4))
            librosa.display.specshow(
                self.log_mel_spec,
                x_axis="time",
                y_axis="mel",
                sr=self.fs)
            plt.colorbar(label="dB")
            plt.title("Log-Mel Spectrogram")
            plt.tight_layout()
            plt.show()