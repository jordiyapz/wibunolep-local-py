from dronekit import connect
import paho.mqtt.client as mqtt
import urllib.parse as urlparse
import serial, time, argparse
from threading import Event, Thread

class RepeatedTimer:

    """Repeat `function` every `interval` seconds."""

    def __init__(self, interval, function, *args, **kwargs):
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.start = time.time()
        self.event = Event()
        self.thread = Thread(target=self._target)
        self.thread.start()

    def _target(self):
        while not self.event.wait(self._time):
            self.function(*self.args, **self.kwargs)

    @property
    def _time(self):
        return self.interval - ((time.time() - self.start) % self.interval)

    def stop(self):
        self.event.set()
        self.thread.join()

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle.
#   Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, heartbeat_timeout=120, wait_ready=True, baud=57600)

# vehicle.wait_ready('autopilot_version')

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("/AsCender/msg")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set('wyyyrgco', 'jpOZK7lVUmlR')
client.connect('soldier.cloudmqtt.com', 14608)
client.publish('/client', 'py-ople');

def assemble_payload(*args):
    payload = ''
    for arg in args:
        payload += str(arg) + ' '
    return payload

def send_payload():
    head = '005'
    v = vehicle
    g = v.location.global_relative_frame
    lat = g.lat
    lon = g.lon
    alt = g.alt
    a = v.attitude
    roll = a.roll
    pitch = a.pitch
    yaw = a.yaw
    heading = v.heading
    airspeed = v.airspeed
    groundspeed = v.groundspeed
    mode = v.mode.name
    armed = v.armed
    payload = assemble_payload(head, alt, lat, lon, roll, pitch, yaw, heading, airspeed, groundspeed, mode, armed)
    print(payload)
    client.publish('/AsCender/payload', payload)

# start timer
timer = RepeatedTimer(1, send_payload);

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.
# client.loop_forever()

try:
    rc = client.loop() # reconnect = false
    while not rc:
        rc = client.loop()
    print("rc: " + str(rc))
except KeyboardInterrupt:
    print('Keyboard interrupt detected!')
finally:
    pass
    # stop timer
    timer.stop()
    print('thread successfully closed')