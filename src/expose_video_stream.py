import cv2
import numpy as np
from flask import Flask, Response

# Number of frames to generate
N = 3

# Define the codec and create VideoWriter objects for each camera
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video_writers = [cv2.VideoWriter(f'output_camera_{i}.avi', fourcc, 20.0, (640, 480)) for i in range(N)]

# Simulating the process that generates N frames
def process_P():
    while True:
        # Simulating frame generation (replace this with your actual frame generation process)
        frames = [np.random.randint(0, 256, size=(480, 640, 3), dtype=np.uint8) for _ in range(N)]
        yield frames

# Initialize the generator
generator = process_P()

app = Flask(__name__)

@app.route('/video_feed/<int:camera_id>')
def video_feed(camera_id):
    def generate():
        while True:
            frame = next(generator)[camera_id]
            ret, jpeg = cv2.imencode('.jpg', frame)
            frame = jpeg.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)