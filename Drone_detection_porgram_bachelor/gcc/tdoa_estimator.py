##### NOT IN USE #########
# THE CLASS WAS AN EARLY CONCEPT, NOT IN USE ANY LONGER.
# USE THE DOA_ESTIMATOR 
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
        self.lag_center = None
        self.lag_samples = None
        self.confidence_score_arr = []


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

    def set_gcc_array(self, gcc_array : np.ndarray):
        self.gcc_array = gcc_array


    def _compute_lags_per_pair(self):
        t_delay_max_arr = [baseline_norm/self.speed_of_sound for baseline_norm in self.baseline_vec_norms]
        max_lag_samples = [int(np.ceil((t_delay_max_pair) * self.gcc_processor.fs)) for t_delay_max_pair in t_delay_max_arr] 
        
        if self.gcc_array is None:
            raise RuntimeError("GCC array is empty, run set_gcc_array in main.py.")
        
        self.n_frames, k_bins = self.gcc_array[0].shape
        self.lag_center = k_bins // 2

        self.lag_min= [max(0,self.lag_center - max_lag_samples_pair) for max_lag_samples_pair in max_lag_samples]
        self.lag_max = [min(k_bins, self.lag_center + max_lag_samples_pair + 1) for max_lag_samples_pair in max_lag_samples]
     
    def _estimate_frame_lags_for_pair(self): 
        if self.gcc_array is None:
            # gcc_array.shape = (pairs, frames, bins)
            raise RuntimeError("GCC array is empty, run set_gcc_array.")
        if self.lag_center is None:
            raise RuntimeError("The lag center is non existant, run _compute_lags_per_pair.")
        
        for pair_indx in range(len(self.gcc_array)):

            lag_min = self.lag_min[pair_indx]
            lag_max = self.lag_max[pair_indx]

            #segment.shape = (n_frames, lag_window)
            segment = np.abs(self.gcc_array[pair_indx][:, lag_min:lag_max])
            #peaks.shape = (n_frames), max value in the segment
            peaks = np.argmax(segment, axis=1)
            
            deltas = np.array([
                _delta_newton_approx(segment[n], peaks[n])
                for n in range(self.n_frames)
            ])

            self.lag_samples = (lag_min + (peaks + deltas)) - self.lag_center
            peak_vals = segment[np.arange(self.n_frames), peaks]
            mean_vals = np.mean(segment.shape)
            confidence = peak_vals/(mean_vals + 1e-15)
            self.confidence_score_arr.append(confidence)

    def _aggregate_frame_delays(self): # Still wrong
        confidence_score_arr = self.confidence_score_arr
        time_delays = self.lag_samples/self.gcc_processor.fs
        keep_frac = 0.5
        k = int(np.ceil(keep_frac * self.n_frames)) # keep 50% of frames
        indx = np.argsort(confidence_score_arr)[-k:] # from the last frame as the first one
        aggregated_time_delays = np.median(time_delays[indx])
        return aggregated_time_delays

    def estimate_TDOA(self):
        self._compute_pair_geometry()
        self._compute_lags_per_pair()
        self._estimate_frame_lags_for_pair()
        return self._aggregate_frame_delays()



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


