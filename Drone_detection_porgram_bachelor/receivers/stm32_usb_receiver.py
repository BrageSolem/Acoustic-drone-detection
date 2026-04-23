import serial
import numpy as np
import time

class STM32UsbReceiver :
    """
    receiver class that holds the following information:
    - Reads framed audio data from Stm32 over USB cdc,
    - Each frame fromat:
     [2 bytes sync] +  [16 samples * 4 channels * 2 bytes],
    - Two first bytes used for alignemnet 
    - Stores data as:
    [[ch1_0, ch1_1, ch1_2 ,..]
     [ch2_0, ch2_1, ch2_2, ..]
     [ch3_0, ch3_1, ch3_2, ...]
     [ch4_0, ch4_1, ch4_2, ...]]
    - each sample is int16 so 2 bytes
    Has The following functions :
    - open_port() which decides which port to read from
    - close_port() to stop the communication through a port
    - record() which records the frames and returns an array of samples 
    """

    def __init__(self, port = "COM16", baud = 921600 , sync= b'\x5A\xA5', channels = 4, frame_samples = 16, duration_sec = 4, linux = False):
        if linux :
            self.port = b'/dev/ttyACM0'
        else :
            self.port = port
        self.baud = baud
        self.sync = sync
        self.channels = channels
        self.frame_samples = frame_samples
        self.duration_sec = duration_sec
        self.ser = None
        self.start_time = None
        self.port_opened = False
        self.samples = [[] for _ in range(self.channels)]
        # each sample = 2 bytes, hence each frame has 16 samples * by 4 channels * by 2 bytes + 2 bytes from sync
        # 130 bytes per frame
        self.frame_bytes = 2 + self.frame_samples * self.channels * 2 
    
    def open_port(self):
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.port_opened = True
    
    def close_port(self):
        if self.port_opened:
            self.ser.close()
            self.port_opened = False
    
    def record(self):
        # remove old data, initiate a temp buffer
        if self.port_opened:
            self.ser.reset_input_buffer() 
            rx_buf = bytearray()

            print("Recording..")
            self.start_time = time.time()
            self.samples = [[] for _ in range(self.channels)] #reset the samples before recording any new values
            
            while time.time() - self.start_time < self.duration_sec :
                data = self.ser.read(512) # 512 bytes
                
                if not data:
                    continue
                rx_buf.extend(data) # adds data to rx_buf

                while True:
                    sync_idx = rx_buf.find(self.sync) 

                    if sync_idx < 0: # if sync was not in the buffer
                        # keep the last byte in case sync 2 bytes are divided between packages
                        rx_buf = rx_buf[-1:]
                        break

                    if len(rx_buf) < sync_idx + self.frame_bytes:
                        # if the length of data is smaller then the expected frame length plus sync bytes
                        break

                    frame = rx_buf[sync_idx + 2 : sync_idx + self.frame_bytes]
                    rx_buf = rx_buf[sync_idx + self.frame_bytes:] # removes the processed frame 

                    samples = np.frombuffer(frame,dtype=np.int16) # interpret as signed int16
                    
                    # vectorized 
                    samples = samples.reshape(-1, self.channels)
                    self.samples = [np.concatenate((self.samples[ch], samples[:,ch])) for ch in range(self.channels)]
                    #for channel in range(self.channels):
                    #    self.samples[channel].extend(samples[channel::self.channels])

            return np.array(self.samples)

        else:
            raise RuntimeError("THE PORT WAS NOT OPENED! Use open_port().")

    
            