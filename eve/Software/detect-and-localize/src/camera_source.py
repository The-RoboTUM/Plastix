import cv2
import glob
import os

IMG_EXTS = ['.jpg', '.jpeg', '.png', '.bmp']
VID_EXTS = ['.avi', '.mov', '.mp4', '.mkv', '.wmv']

class CameraSource:
    def __init__(self, source, resolution=None):
        self.source = source
        self.resolution = resolution
        self.source_type = None
        self.cap = None
        self.imgs_list = []
        self.img_count = 0
        self._init_source()

    def _init_source(self):
        if os.path.isdir(self.source):
            self.source_type = 'folder'
            self.imgs_list = [f for f in glob.glob(self.source + '/*') if os.path.splitext(f)[1].lower() in IMG_EXTS]
        elif os.path.isfile(self.source):
            ext = os.path.splitext(self.source)[1].lower()
            if ext in IMG_EXTS:
                self.source_type = 'image'
                self.imgs_list = [self.source]
            elif ext in VID_EXTS:
                self.source_type = 'video'
                self.cap = cv2.VideoCapture(self.source)
        elif 'usb' in self.source:
            self.source_type = 'usb'
            idx = int(self.source[3:])
            self.cap = cv2.VideoCapture(idx)
        elif self.source.startswith('rtsp://') or self.source.startswith('http'):
            self.source_type = 'stream'
            self.cap = cv2.VideoCapture(self.source)
        else:
            raise ValueError(f'Invalid source: {self.source}')

        if self.cap and self.resolution:
            w, h = map(int, self.resolution.split('x'))
            self.cap.set(3, w)
            self.cap.set(4, h)

    def read(self):
        if self.source_type in ['image', 'folder']:
            if self.img_count >= len(self.imgs_list):
                return None
            frame = cv2.imread(self.imgs_list[self.img_count])
            self.img_count += 1
            return frame
        else:
            ret, frame = self.cap.read()
            return frame if ret else None

    def release(self):
        if self.cap:
            self.cap.release()
