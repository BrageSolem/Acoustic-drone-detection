from gcc.doa_estimator import TDOAEstimator
from gcc.doa_estimator import DOAEstimator
from gcc.gcc_processor import GCCProcessor
import numpy as np


class GCCDebug:
    """
    The class is used for debuging the gcc. Curently it only allows for change of the position
    vector of the microphones.
    
    """

    def __init__(self, tdoa_estimator : TDOAEstimator, doa_estimator :DOAEstimator, gcc_processor : GCCProcessor ):
        self.tdoa_estimator = tdoa_estimator
        self.doa_estimator = doa_estimator
        self.gcc_processor = gcc_processor        

    def change_p_vector(self, new_p_vector : np.ndarray):
        self.p_vector =  new_p_vector  
