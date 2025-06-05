import cv2
import numpy as np
from openni import _openni2 as c_api
from openni import openni2


class Camera:
    def __init__(self, 
                 fps=30, 
                 width=640, 
                 height=480, 
                 openni_libs='/home/a/Downloads/libs'):
        #config default parameters.
        self.fps = fps
        self.width = width
        self.height = height
        self.openni_libs = openni_libs
        self.wait_time = int(1000.0/float(fps))
        self.load()
    
    def unload(self):
        openni2.unload()
        
    def load(self):
            #setting up openni and chose device.
            print(123)
            openni2.initialize(self.openni_libs)
            self.dev = openni2.Device.open_any()
            # init depth stream.
            self.depth_stream = self.dev.create_depth_stream()
            self.depth_stream.start()
            self.depth_stream.set_video_mode(
                c_api.OniVideoMode(
                    pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_DEPTH_1_MM, 
                    resolutionX = self.width, 
                    resolutionY = self.height, 
                    fps = self.fps
                )
            )
            # init color stream.
            self.color_stream = self.dev.create_color_stream()
            self.color_stream.start()
            self.color_stream.set_video_mode(
                c_api.OniVideoMode(
                    pixelFormat = c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, 
                    resolutionX = self.width, 
                    resolutionY = self.height, 
                    fps = self.fps
                )
            )
            # register both streams.
            self.dev.set_image_registration_mode(
                c_api.OniImageRegistrationMode.ONI_IMAGE_REGISTRATION_DEPTH_TO_COLOR
            )
        
    def get_depth(self):
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()
        img = np.ndarray((frame.height, frame.width), dtype=np.uint16, buffer=frame_data)
        return img
    
    def get_color(self):
        frame = self.color_stream.read_frame()
        frame_data = frame.get_buffer_as_uint8()
        colorPix = np.frombuffer(frame_data, dtype=np.uint8)
        colorPix.shape = (self.height, self.width, 3)
        return colorPix
    
    def get_depth_and_color(self):
        return self.get_depth(), self.get_color()
    
