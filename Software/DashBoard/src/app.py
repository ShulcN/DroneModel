from flask import Flask, render_template, request, jsonify
import json

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/mqtt_config', methods=['POST'])
def mqtt_config():
    # This endpoint will be used to pass MQTT configuration to the frontend
    data = request.json
    # In a real application, you might validate the config here
    return jsonify({"status": "success"})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')