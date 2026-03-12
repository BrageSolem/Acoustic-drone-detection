import pandas as pd
from mfcc_extraction import MFCCExtractor

class FeatureExporter:
        def __init__(self,extractor: MFCCExtractor ):
            self.extractor = extractor

            if not isinstance(extractor, MFCCExtractor):
                 raise TypeError("extracotr must be of MFCCExtractor")
            
        def df_features(self, filename="logmel_features.csv"):

            if self.extractor.acc_features is None:
                raise ValueError("No features available. Run Run extract_features() from mfcc_extraction")

            feature_names = (
                [f"mel_mean_{i}" for i in range(self.extractor.n_mels)] +  # Avg log-energy in freq band over the whole recording, low value - low freq, high value high freq
                [f"mel_std_{i}"  for i in range(self.extractor.n_mels)] + # How stable the energy is, low value steady mechanical sound, otherwise highly modulated sound such as voice
                [f"mfcc_mean_{i}" for i in range(self.extractor.n_mfcc)] + # Spectral shape, different source different envelope
                [f"mfcc_std_{i}"  for i in range(self.extractor.n_mfcc)] #stability of the spectral envelope, low vlaue - stable - drone / high value - unstable - speach
            )

            df = pd.DataFrame([self.extractor.acc_features], columns=feature_names)
            df.to_csv(filename, index=False) 
            return df