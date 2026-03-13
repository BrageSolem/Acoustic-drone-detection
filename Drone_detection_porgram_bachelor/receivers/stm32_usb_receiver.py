import serial
import numpy as np
import pandas as pd
import time
import matplotlib.pyplot as plt



class STM32UsbReceiver :
    def __init__(self, port = "COM6", baud = 921600 , sync= b'\x5A\xA5', channels = 4, frame_samples = 16, duration_sec = 1, linux = False):
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

                    for channel in range(self.channels):
                        self.samples[channel].extend(samples[channel::self.channels])

            return self.samples

        else:
            raise RuntimeError("THE PORT WAS NOT OPENED! Use open_port().")

    
            