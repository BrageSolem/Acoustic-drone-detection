import pandas as pd

df = pd.read_csv("testing/logmel_features.csv")

ok = False
i = 0 
while not ok:# 40 = self.n_mels , 13 = self.n_mfcc , Should not be hardcoded but fck for not, it is just a test

    if i < 40: # depends on the number of mels filters
        print(df[f"mel_mean_{i}"]) # Avg log-energy in freq band over the whole recording, low value - low freq, high value high freq
        print(df[f"mel_std_{i}"]) # How stable the energy is, low value steady mechanical sound, otherwise highly modulated sound such as voice
    if i < 13: # depends on the number of mfcc coefficients
        print(df[f"mfcc_mean_{i}"])  # Spectral shape, different source different envelope
        print(df[f"mfcc_std_{i}"])#stability of the spectral envelope, low vlaue -stable -drone / high value -unstable -speach
    i += 1
    if i == 40:
        ok = True
    