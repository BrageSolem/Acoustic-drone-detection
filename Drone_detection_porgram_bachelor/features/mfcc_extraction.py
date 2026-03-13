import numpy as np
import librosa


class MFCCExtractor:
    def __init__(self, n_mfcc = 13, n_fft = 1024, hop_length = 250, win_length = 500, n_mels = 40):
        #####
        # CHANGE THE default values to suit the actual fs!!!!
        ##### 

        self.n_mfcc = n_mfcc
        self.n_fft = n_fft
        self.hop_length = hop_length
        self.win_length = win_length
        self.n_mels = n_mels

        self.signal = None
        self.fs = None
        self.mfcc = None
        self.mel_spec = None
        self.log_mel_spec = None
        self.acc_features =None
        self.signal_time_domain = None
        

    def _convert_wav_to_signal(self, audio_file):
        self.signal, self.fs = librosa.load(audio_file, sr = None)
        self.signal_time_domain = self.signal

    def _get_preemphasised_signal(self):
        return librosa.effects.preemphasis(self.signal)
    
    def _mfcc_after_preemphasis(self):
        signal_preemphasised = self.get_preemphasised_signal()

        self.mfcc = librosa.feature.mfcc(
            y = signal_preemphasised,
            sr = self.fs,
            n_mfcc= self.n_mfcc,
            n_fft = self.n_fft,
            hop_length = self.hop_length,
            win_length = self.win_length,
            n_mels = self.n_mels
        )

    def _get_mfcc_features(self):
        if self.signal is not None:
            mfcc_delta = librosa.feature.delta(self.mfcc)
            mfcc_delta2 = librosa.feature.delta(self.mfcc, order=2)
            return np.vstack([self.mfcc, mfcc_delta, mfcc_delta2])
        else:
            raise ValueError("The signal is empty! Run mfcc_after_preemphasis()")
    
    def _create_mel_spec(self):
        self.mel_spec = librosa.feature.melspectrogram(
            y = self.signal,
            sr = self.fs,
            n_fft= self.n_fft,
            hop_length=self.hop_length,
            win_length=self.win_length,
            n_mels = self.n_mels,
            power = 2
        )

    def _power_mel_spec(self):
        self.log_mel_spec = librosa.power_to_db(self.mel_spec, ref=np.max)

    def _accumulate_the_stats(self):
        logmel_stats = np.hstack([self.log_mel_spec.mean(axis=1), self.log_mel_spec.std(axis=1)])
        mfcc_stats = np.hstack([self.mfcc.mean(axis=1),self.mfcc.std(axis=1)])
        self.acc_features = np.hstack([logmel_stats,mfcc_stats])

# public
    def extract_features(self, audio_file):
        self.convert_wav_to_signal(audio_file)
        self.mfcc_after_preemphasis()
        self.create_mel_spec()
        self.power_mel_spec()
        self.accumulate_the_stats()
        return self.acc_features
