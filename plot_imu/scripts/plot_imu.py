#!/usr/bin/env python
import socketio
import eventlet
from flask import Flask, render_template
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64

def init_plotter():
    rospy.init_node('plot_imu', anonymous=True)

sio = socketio.Server(logger=False, engineio_logger = False)
app = Flask(__name__)

@app.route('/')
def index():
    """Serve the client-side application."""
    # return render_template('index.html')

@sio.on('connect')
def connect(sid, environ):
    print('connect ', sid)

x_acc_pub = rospy.Publisher('imu_acc_x', Float64, queue_size=10)
y_acc_pub = rospy.Publisher('imu_acc_y', Float64, queue_size=10)
z_acc_pub = rospy.Publisher('imu_acc_z', Float64, queue_size=10)

@sio.on('imu')
def message(sid, data):
    x_acc_pub.publish(data[0])
    y_acc_pub.publish(data[1])
    z_acc_pub.publish(data[2])

@sio.on('disconnect')
def disconnect(sid):
    print('disconnect ', sid)

if __name__ == '__main__':
    init_plotter()

    # wrap Flask application with socketio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 3000)), app)

    #while not rospy.is_shutdown():
        #get_velocity()
