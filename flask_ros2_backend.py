from threading import Thread
import io
from flask import Flask, request, send_file, jsonify
from flask_cors import CORS
import subprocess
import os
from datetime import datetime
import shutil
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
import logging
from frontend_image_subscriber import FrontendImageSubscriber
from logging.handlers import RotatingFileHandler

app = Flask(__name__)
CORS(app)
recording_process = None

image_subscriber = None

@app.route('/start', methods=['POST'])
def start_recording():
    global recording_process
    now = datetime.now()
    date_path = now.strftime('/data/%Y-%m-%d')
    time_filename = now.strftime('%H:%M:%S')
    os.makedirs(date_path, exist_ok=True)
    output_path = f'{date_path}/{time_filename}.bag'
    recording_process = subprocess.Popen(['ros2', 'bag', 'record', '-a', '--output', output_path])
    return f'Started recording, saving to {output_path}', 200

@app.route('/stop', methods=['POST'])
def stop_recording():
    global recording_process
    if recording_process:
        recording_process.send_signal(subprocess.signal.SIGINT)
        recording_process = None
    return 'Stopped recording', 200

@app.route('/disk-space', methods=['GET'])
def check_disk_space():
    disk_path = request.args.get('path', '/')
    try:
        usage = shutil.disk_usage(disk_path)
        return {
            'total': round(usage.total / (1024 ** 3), 2),
            'used': round(usage.used / (1024 ** 3), 2),
            'free': round(usage.free / (1024 ** 3), 2)
        }, 200
    except Exception as e:
        return str(e), 500

@app.route('/latest-image', methods=['GET'])
def get_latest_image():
    global image_subscriber
    if image_subscriber.latest_image is not None:
        return send_file(
            io.BytesIO(image_subscriber.latest_image),
            mimetype='image/jpeg',
            as_attachment=True,
            download_name='latest.jpg')
    else:
        return 'No image available', 404

@app.route('/ros/topics')
def get_ros_topics():
    try:
        topics = subprocess.check_output(['ros2', 'topic', 'list'], text=True)
        topics_list = topics.strip().split('\n')
        return jsonify(topics_list)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/ros/echo', methods=['POST'])
def echo_ros_topic():
    topic_name = request.json.get('topic')
    if not topic_name:
        return jsonify({"error": "No topic name provided"}), 400
    try:
        # Using subprocess to execute the ros2 command and get the first few lines
        echo_output = subprocess.check_output(['ros2', 'topic', 'echo', '--once', topic_name], text=True, timeout=10)
        return jsonify({"data": echo_output})
    except subprocess.TimeoutExpired:
        return jsonify({"error": f"Timeout: Could not get data from {topic_name}."}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500



def run_image_subscriber():
    rclpy.init(args=None)
    global image_subscriber
    image_subscriber = FrontendImageSubscriber("/camera_image/cam_HR")
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # Creating threads for both Flask app and ImageSubscriber
    thread_subscriber = Thread(target=run_image_subscriber)
    thread_subscriber.start()

    app.run(host='0.0.0.0', port=5000, debug=True, threaded=False)
