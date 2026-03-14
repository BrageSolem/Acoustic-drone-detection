import numpy as np
from gcc.gcc_processor import GCCProcessor

class TDOAEstimator():
    def __init__(self, p_vector : np.ndarray, gcc_processor : GCCProcessor):
        self.p_vector = p_vector 
        self.gcc_processor = gcc_processor


    def _find_baseline_vec_normalized(self, cps_array : np.ndarray):
        
        if self.gcc_processor.mic_pairs is None:
            mic_pairs = self.gcc_processor.mic_pairs

        for i,j in mic_pairs:
            baseline_vec = self.p_vector[:, i] - self.p_vector[:,j]
            baseline_vec_norm = np.linalg.norm(baseline_vec)
        return baseline_vec_norm
    
        # SHould i continue with find the time delay here?


    










def _delta_newton_approx(y,k):
        if k <= 0 or k >= len(y) - 1:
            return 0

        y0 = y[k - 1] 
        y1 = y[k]
        y2 = y[k + 1]

        b = 0.5 * (y2 - y0) # centered approx of f'(x) when h = 1

        a = y2 + y0 - 2*y1
        if np.abs(a) < 1e-15: # in case a is (almost) 0, to prevent division by 0
            return 0
    
        delta = -b / a
        return float(delta)


n_frames, n_pairs, k_bins = gcc_array.shape