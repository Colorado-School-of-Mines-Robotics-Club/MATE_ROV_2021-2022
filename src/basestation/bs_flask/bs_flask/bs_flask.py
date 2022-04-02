import base64
import cv2
from flask import Flask, request, render_template, render_template_string, Response
import numpy
import os
import socket
from threading import Thread, Lock
import time

DEFAULT_IMAGE = "default.jpg"

app = Flask(__name__)
video_capture = cv2.VideoCapture(0)

class ImageServer:
    def __init__(self) -> None:
        self.thread = Thread(target=self._run, args=(self,), daemon=True, name="ImageServer")
        self.address = ("0.0.0.0", 7777)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mutex = Lock()
        self.html_img = self.createDefaultImage()

    def createDefaultImage(self):
        return self.convertNumpyArray(cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), DEFAULT_IMAGE)))

    def convertNumpyArray(self, ndarray):
        _, buffer = cv2.imencode('.jpg', ndarray)
        return buffer.tobytes() # readable by html-img tag

    def getImage(self):
        self.mutex.acquire() # ensure no writes while copying (adds some delay to the output stream but probably worth it)
        copy = self.html_img
        self.mutex.release()
        return copy # return image

    def _run(self, *args, **kwargs):
        # self.s.bind(self.address)
        # sock, addressinfo = self.s.accept()
        while True:
            # incomingImage = sock.recv(4096)
            # ndarray = convertBytesToNDArray(incomingImage) # convert incoming image to numpy ndarray
            ndarray = video_capture.read()[1] # testing purposes (my laptop cam is 1280x720 or 921,600 pixels)
            new_html = self.convertNumpyArray(ndarray) # assumes ndarray in RGB order already
            self.mutex.acquire() # we want to hold this mutex for as little time as possible
            self.html_img = new_html
            self.mutex.release()

def gen():
    while True:
        yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + videoServer.getImage() + b'\r\n\r\n')

@app.route('/')
def index():
    """Video streaming"""
    return render_template('index.html')

@app.route('/camera_1_video_feed')
def camera_1_video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return
    return Response(gen(),
                mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/shell")
def shellCommand():
    recieved_command = request.args.get("command")
    print("Recieved command from JS: {}".format(recieved_command))

    try:
        recieved_command_result = eval(recieved_command)
    except Exception as e:
        recieved_command_result = e

    print("Sending command result to JS: {}".format(recieved_command_result))
    return {"result": recieved_command_result}

if __name__ == '__main__':
    global videoServer
    # videoServer = ImageServer()
    # time.sleep(0.25)
    # videoServer.thread.start()
    app.run()
