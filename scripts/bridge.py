import Message_cvMat
import cv2

def get_cvmat_names():
    return Message_cvMat.names()

def get_cvmat_jpegcode(name):
    wi = Message_cvMat.Subscriber(name, 1)
    if wi is not None:
        while True:
            jpeg_code = cv2.imencode(".jpeg", wi.pop().get_nparray())[1].tobytes()
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg_code + b'\r\n\r\n'


