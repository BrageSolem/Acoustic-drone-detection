def _cross_power_spectrum(self,frames_fft:np.ndarray):
        cps = []
        n_frames = frames_fft.shape[0]
        n_mics = frames_fft.shape[1]

        if self.mic_pairs in None:
            self.mic_pairs = self._generate_mic_pairs(n_mics)
        
        mic_pairs = self.mic_pairs

        # for each frame multi mic_i with the conjugate of mic_j

        for frame in range(n_frames): 
            frame_cps = [] # cps of one frame
            for i,j in mic_pairs:
                cps_ij = frames_fft[frame, i, :] *frames_fft[frame,j,:].conj()
                frame_cps.append(cps_ij)
            cps.append(frame_cps)

        # frame_cps shape (frame, mic pair, cps)
        return np.array(cps)