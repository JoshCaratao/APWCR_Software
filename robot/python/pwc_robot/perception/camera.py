import cv2
import platform

class Camera:
    def __init__(self, index: int = 0, width: int | None = None, height: int | None = None):
        """
        Simple camera wrapper.
        index: which camera to open (0 = default)
        width, height: optional resolution hints
        """
        self.index = index
        self.width = width
        self.height = height
        self.cap = None

    # Open Camera Function
    def open(self) -> bool:
        if platform.system() == "Windows":
            # Use DirectShow backend on Windows to avoid long hangs
            self.cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(self.index)

        if not self.cap.isOpened():
            print(f"Error: Could not open camera {self.index}.")
            return False

        if self.width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        return True
    
    #Check if camera is opened
    @property
    def is_open(self) -> bool:
        """
        Returns True if the camera is currently opened.
        """
        return self.cap is not None and self.cap.isOpened()

    
    # Read Camera Frames
    def read(self):
        """
        Returns (ret, frame) like cv2.VideoCapture.read().
        """
        if self.cap is None:
            raise RuntimeError("Camera not opened. Call open() first.")
        return self.cap.read()
    
    # Get what resolution my camera is giving
    def get_resolution(self) -> tuple[int, int]:
        if self.cap is None:
            raise RuntimeError("Camera not opened.")
        w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h


    # Release Camera
    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
