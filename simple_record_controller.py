from flask import Flask, request
from flask_cors import CORS
import subprocess
import os
from datetime import datetime
import shutil

app = Flask(__name__)
CORS(app)
recording_process = None  # Global variable to keep track of the recording process

@app.route('/start', methods=['POST'])
def start_recording():
    global recording_process
    # Get current date and time
    now = datetime.now()
    date_path = now.strftime('/data/%Y-%m-%d')
    time_filename = now.strftime('%H:%M:%S')  # Added seconds to ensure unique filenames
    
    # Create directory if it doesn't exist
    os.makedirs(date_path, exist_ok=True)
    
    # Start recording, specifying the full path
    output_path = f'{date_path}/{time_filename}.bag'  # Added file extension for clarity
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
        free_space_gb = usage.free / (1024 ** 3)  # Convert from bytes to gigabytes
        return {
            'total': round(usage.total / (1024 ** 3), 2),
            'used': round(usage.used / (1024 ** 3), 2),
            'free': round(free_space_gb, 2)  # Free space in gigabytes, rounded to two decimal places
        }, 200
    except Exception as e:
        return str(e), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
