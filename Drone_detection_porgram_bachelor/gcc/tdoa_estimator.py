import numpy as np
from gcc.gcc_processor import GCCProcessor

class TDOAEstimator():
    def __init__(self, p_vector : np.ndarray, gcc_processor : GCCProcessor):
        self.p_vector = p_vector 
        self.gcc_processor = gcc_processor
        self.speed_of_sound = 343
        self.baseline_vecs = None
        self.baseline_vec_norms = None
        self.gcc_array = None
        self.lag_min = None
        self.lag_max = None
        self.n_frames = None

    def _compute_pair_geometry(self):
        # Finds the baseline vectors, normalize them and creates the baseline unit vectors
        if self.gcc_processor.mic_pairs is None:
            raise RuntimeError("No microphone pairs found, and probably the cross power spectrum is also absent. Run gcc_processor.process_signal in main.py")

        mic_pairs = self.gcc_processor.mic_pairs
        
        self.baseline_vecs = []
        self.baseline_vec_norms = []
        
        for i,j in mic_pairs:
            baseline_vec_ij = self.p_vector[:, i] - self.p_vector[:,j]
            self.baseline_vecs.append(baseline_vec_ij)
            baseline_vec_norm = np.linalg.norm(baseline_vec_ij)
            self.baseline_vec_norms.append(baseline_vec_norm)

    def set_gcc_array(self, gcc_array):
        self.gcc_array = gcc_array

    def _compute_lags_per_pair(self):
        t_delay_max_arr = [baseline_norm/self.speed_of_sound for baseline_norm in self.baseline_vec_norms]
        max_lag_samples = [int(np.ceil((t_delay_max_pair) * self.gcc_processor.fs)) for t_delay_max_pair in t_delay_max_arr] 
        
        if self.gcc_array is None:
            raise RuntimeError("GCC array is empty, run set_gcc_array.")
        
        self.n_frames, k_bins = self.gcc_array[0].shape
        lag_center = k_bins // 2

        self.lag_min= [max(0,lag_center - max_lag_samples_pair) for max_lag_samples_pair in max_lag_samples]
        self.lag_max = [min(k_bins, lag_center + max_lag_samples_pair + 1) for max_lag_samples_pair in max_lag_samples]

    
    def _interpolate_peaks(self, frames, peaks):


     
    def _estimate_frame_lags_for_pair(self): # Has to add 
        if self.gcc_array is None:
            raise RuntimeError("GCC array is empty, run set_gcc_array.")
        

        
        peaks_frames = [np.argmax(frame) for frame in self.gcc_array[:,self.lag_min : self.lag_max]]  # WRONG
        interpolated_peaks = self._interpolate_peaks(self.gcc_array, peaks_frames)


    def _aggregate_frame_delays(self):



    def estimate_TDOA(self):
        self._compute_pair_geometry()
        self._compute_lags_per_pair()









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