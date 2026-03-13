from mfcc_extraction import MFCCExtractor
import matplotlib.pyplot as plt
import librosa.display


class FeatureVisualizer:
    def __init__(self, extractor: MFCCExtractor):
        self.extractor = extractor
            
    def plot_signal_time(self):
        signal = self.extractor.signal_time_domain
        if signal is not None:
            plt.figure(figsize=(10,4))
            plt.plot(signal)
            plt.xlabel("Time (s)")
            plt.ylabel("Amplitude")
            plt.title("Signal plot (time–amplitude view)")
            plt.tight_layout()
            plt.show()

    def spectrogram(self):
        if self.extractor.signal is not None:
            plt.figure(figsize=(10, 4))
            plt.specgram(self.extractor.signal,
                         self.extractor.n_fft, 
                         self.extractor.fs, 
                         noverlap=self.extractor.win_length - self.extractor.hop_length,
                         cmap = "magma")
                
            plt.xlabel("Time (s)")
            plt.ylabel("Frequency (Hz)")
            plt.title("Spectrogram (time–frequency view)")
            plt.colorbar(label="Power")
            plt.tight_layout()
            plt.show()
        else:
            raise RuntimeError("Signal not loaded. Run extract_features() first.")

    def mel_spectrogram(self):
        if self.extractor.log_mel_spec is not None:
            plt.figure(figsize=(10, 4))
            librosa.display.specshow(
                self.extractor.log_mel_spec,
                x_axis="time",
                y_axis="mel",
                sr=self.extractor.fs)
            plt.colorbar(label="dB")
            plt.title("Log-Mel Spectrogram")
            plt.tight_layout()
            plt.show()
        else:
            raise RuntimeError("Signal not loaded. Run extract_features() first.")