import numpy as np
from gcc.gcc_processor import GCCProcessor

class DOAEstimator():
    """
    The class estimated the direction of arrival (DOA) of a sound source using time difference of arraival (TDOA) derived from GCC processing

    The program flow is as follows:
    - compute geometry for the microphones by finding the baseline vectors between each microphone,
    - set the gcc array from gcc_processor,
    - For each microphone pair:
        - limit the lag search range based on the physical constraints of the system,
        - For each frame:
            - Find the peak of the gcc function, which represents the estimated delay,
            - Apply delta_newton interpolation to estimate the actual peak in proximity of the integer peak,
            - Compute the confidence score based on the ratio between the segmnet peak value, and the avg values present in the segment,
        - Keep 50% of the most reliable frames  in the segment,
        - Aggregate the delays,
    - Convert TDOAs into cone angles,
    - Perform a grid search over possible azimuth agnles:
        - COmpare predicted TDOAs with measured TDOAs,
        - Select the angle that minimizes the total error  
    
    The result is an estimated azimuth direction of the sound source.
    
    """    



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
        self.aggregated_time_delay_list = None
        self.cone_angles = None
        self.estimated_azimuth_rad = None
        self.estimated_azimuth_deg = None


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
        
        self.n_frames = self.gcc_array.shape[0]
        k_bins = self.gcc_array.shape[2]
        self.lag_center = k_bins // 2

        self.lag_min= [max(0,self.lag_center - max_lag_samples_pair) for max_lag_samples_pair in max_lag_samples]
        self.lag_max = [min(k_bins, self.lag_center + max_lag_samples_pair + 1) for max_lag_samples_pair in max_lag_samples]
    
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

    def _estimate_tdoa(self): 
        if self.gcc_array is None:
            # gcc_array.shape = ( frames, pairs,  bins)
            raise RuntimeError("GCC array is empty, run set_gcc_array.")
        if self.lag_center is None:
            raise RuntimeError("The lag center is non existant, run _compute_lags_per_pair.")
        
        self.aggregated_time_delay_list = []
        self.cone_angles  = []
        keep_frac = 0.5
        fs = self.gcc_processor.fs

        for pair_indx in range(len(self.gcc_array.shape[1])):

            lag_min = self.lag_min[pair_indx]
            lag_max = self.lag_max[pair_indx]

            
            segment = np.abs(self.gcc_array[:, pair_indx, :][:, lag_min:lag_max])
            
            peaks = np.argmax(segment, axis=1)
            
            deltas = np.array([
                self._delta_newton_approx(segment[n], peaks[n])
                for n in range(self.n_frames)
            ])

            lag_samples = (lag_min + (peaks + deltas)) - self.lag_center
            peak_vals = segment[np.arange(self.n_frames), peaks]
            mean_vals = np.mean(segment, axis = 1)
            confidence = peak_vals/(mean_vals + 1e-15)


            time_delays = lag_samples/fs 
            
            k = max(1, int(np.ceil(keep_frac * self.n_frames))) # keeps 50% of frames
            indx = np.argsort(confidence)[-k:] # sorts all of the frames from lowest to the highest confidency score, and keeps the last k elements, which are the highest 

            aggregated_time_delay = np.median(time_delays[indx])
            self.aggregated_time_delay_list.append(aggregated_time_delay)
            
            baseline_vec_norm = self.baseline_vec_norms[pair_indx]

            cos_arg = np.clip((self.speed_of_sound * aggregated_time_delay) / baseline_vec_norm, -1, 1) # must ensure the cos will not be bigger or smaller then 1 or -1 
            cone_angle_rad = np.arccos(cos_arg)
            self.cone_angles.append(cone_angle_rad)

            # Consider adding a filter to the angle, if the angle will be too unstable

        self.aggregated_time_delay_list = np.array(self.aggregated_time_delay_list)
        self.cone_angles = np.array(self.cone_angles)

    def _estimate_azimuth(self): # only finds the azimuth of the target, not the elevation
        if self.aggregated_time_delay_list is None:
            raise RuntimeError("Time delyas are empty, Run _estimate_frame_lags_for_pair first")

        azimuth_grid_deg = np.linspace(-180, 180, 3601) # 3601 give 0.1 resolution, otherwise use 360 to get 1 degree res
        azimuth_grid_rad = np.radians(azimuth_grid_deg)

        R_test = 1 # arbitrary radius (far -field approxiation)
        error_surface = []
        mic_pairs = self.gcc_processor.mic_pairs

        for phi in azimuth_grid_rad:
        # hyphotesized source position q 
            u = np.array([np.cos(phi), np.sin(phi),0])
            q = R_test * u
            pair_errors = []

            for pair_indx, (i,j) in enumerate(mic_pairs):
            # eq: 8.10
                predicted_tdoa = (np.linalg.norm(q - self.p_vector[:, i]) - np.linalg.norm(q - self.p_vector[:, j]))/self.speed_of_sound

            #eq: 8.12 
                error = (self.aggregated_time_delay_list[pair_indx] - predicted_tdoa)**2
                pair_errors.append(error)

            error_surface.append(np.sum(pair_errors))
        error_surface = np.array(error_surface)

        best_indx = np.argmin(error_surface)
        self.estimated_azimuth_rad = azimuth_grid_rad[best_indx]
        self.estimated_azimuth_deg = azimuth_grid_deg[best_indx]

    def estimate_DOA(self):
        self._compute_pair_geometry()
        self._compute_lags_per_pair()
        self._estimate_tdoa()
        self._estimate_azimuth()




#

