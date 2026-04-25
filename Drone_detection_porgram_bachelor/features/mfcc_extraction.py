import numpy as np
import librosa


class MFCCExtractor:
    """
    The class processes a WAV audio file and extracts based on MFCC and log-Mel spectrogram representation.

    THe process has the following steps:
    - Load audio signal from a wav file,
    - Preemphesis the signal to boost the high freqency components relative to lower freqs. Used in case the blade noise lies in higher bands,
    - Find MFCC coeffs that represents the spectral envelope in Mel scale,
    - Compute spectrogram and convert it to a log-Mel spectrogram (used in debug to visualize the signal)
    - Aggregate mean and std of log-mel and mfcc

    Returns:

    - Feature vector of:
        - Mean and std of log-Mel spectrogram
        - Mean and std of MFCC coefficients

    Additional notes:
    - Mfcc captures the spectral shape of the signal
    - Delta and Delta2 represents the dynamics of the spectral envelope
    - The deltas are currently not used for drone detection, but they might come in handy, if the project will include
    speach detection or enviromental noise
    - MFCC - compressed spectral shape 
    - low-mel - actual energy distribution across freq bands in db scale
    """



    def __init__(self, n_mfcc = 40, n_fft = 1024, hop_length = 250, win_length = 500, n_mels = 40):
        #####
        # CHANGE THE default values to suit the actual fs!!!!
        ##### 

        self.n_mfcc = n_mfcc # number of coefficients
        self.n_fft = n_fft # size of FFT, larger gives finner freq resolution, but also more computation
        self.hop_length = hop_length # number of samples we move forward between consecutive frames, by default 50% overlap
        self.win_length = win_length # number of samples in a window
        self.n_mels = n_mels # number of mel filters 
 
        self.signal = None # 1D array of signal
        self.fs = None # sampling rate
        self.mfcc = None # 2D array of MFCC, (n_mfcc, number of frames)
        self.mel_spec = None # 2D array of mel spectogram (n_mels, number of frames)
        self.log_mel_spec = None # same form as above, but transformed into power (db) scale 
        self.acc_features =None # 1D array of accumulated features (mean and stds )
        self.signal_time_domain = None # 1D array of original signal, before the preemphesis (used to debug)
        

    def _convert_wav_to_signal(self, audio_file):
        self.signal, self.fs = librosa.load(audio_file, sr = None)
        self.signal_time_domain = self.signal

    def _get_preemphasised_signal(self):
        return librosa.effects.preemphasis(self.signal)
    
    def _mfcc_after_preemphasis(self):
        signal_preemphasised = self._get_preemphasised_signal()

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
        if self.mfcc is not None:
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
        # mffcc mean - avg spectral shape, or how the sound looks like in the spectrum. Is it more "flat" or "steep" or "curved"
        # mfcc std - how much the whole spectral envelope changes over time 

        # log-mel mean - how the energy is distirbuated across freq bands (in db)
        # low-mel std - how much energy in each band changes over time

        logmel_stats = np.hstack([self.log_mel_spec.mean(axis=1), self.log_mel_spec.std(axis=1)])
        mfcc_stats = np.hstack([self.mfcc.mean(axis=1),self.mfcc.std(axis=1)])
        self.acc_features = np.hstack([logmel_stats,mfcc_stats])

# public
    def extract_features(self, audio_file):
        self._convert_wav_to_signal(audio_file)
        self._mfcc_after_preemphasis()
        self._create_mel_spec()
        self._power_mel_spec()
        self._accumulate_the_stats()
        return self.acc_features
