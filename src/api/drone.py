from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative
from pymavlink import mavutil
import time
import config
import json

class Drone():
  identifier = "XXXX"
  connection_string = "/dev/ttyUSB0"
  baud_rate = 115200
  vehicle = None

  def __init__(self, identifier="XXXX", connection_string="/dev/ttyUSB0", baud_rate=115200):
    self.identifier = identifier
    self.connection_string = connection_string
    self.baud_rate = baud_rate

  def __str__(self):
    return "Drone: %s (c: %s, b: %d)" % (self.identifier,
       self.connection_string,
       self.baud_rate)

  def config(self, config):
    self.identifier = config.IDENTIFIER
    self.connection_string = config.CONNECTION_STRING
    self.baud_rate = config.BAUD_RATE

  def radio_status(self, attr_name, value):
    print "RADIO_STATUS: %s" % value

  def connect(self):
    self.vehicle = connect(self.connection_string, wait_ready=True,
                           baud=self.baud_rate)
    return "CONNECTED"


  def disconnect(self):
    if self.vehicle:
      self.vehicle.close()
      self.vehicle = None
    return "DISCONNECTED"

  def get_attributes(self):
    if self.vehicle == None:
      return "DISCONNECTED"

    attributes = json.loads("{}")
    attributes['version'] = str(self.vehicle.version)
    attributes['groundspeed'] = self.vehicle.groundspeed
    attributes['airspeed'] = self.vehicle.airspeed
    attributes['heading'] = self.vehicle.heading
    attributes['last_hearbeat'] = self.vehicle.last_heartbeat
    attributes['is_armable'] = self.vehicle.is_armable
    attributes['armed'] = self.vehicle.armed
    attributes['ekf_ok'] = self.vehicle.ekf_ok
    attributes['velocity'] = self.vehicle.velocity

    system_status = json.loads("{}")
    system_status['state'] = self.vehicle.system_status.state
    attributes['system_status'] = system_status

    mode = json.loads("{}")
    mode['name'] = self.vehicle.mode.name
    attributes['mode'] = mode

    capabilities = json.loads("{}")
    capabilities['ftp'] = self.vehicle.capabilities.ftp
    attributes['capabilities'] = capabilities

    location = json.loads("{}")
    global_frame = json.loads("{}")
    global_frame['lat'] = self.vehicle.location.global_frame.lat
    global_frame['lon'] = self.vehicle.location.global_frame.lon
    global_frame['alt'] = self.vehicle.location.global_frame.alt
    location['global_frame'] = global_frame

    global_relative_frame = json.loads("{}")
    global_relative_frame['lat'] = self.vehicle.location.global_relative_frame.lat
    global_relative_frame['lon'] = self.vehicle.location.global_relative_frame.lon
    global_relative_frame['alt'] = self.vehicle.location.global_relative_frame.alt
    location['global_relative_frame'] = global_relative_frame

    local_frame = json.loads("{}")
    local_frame['north'] = self.vehicle.location.local_frame.north
    local_frame['east'] = self.vehicle.location.local_frame.east
    local_frame['down'] = self.vehicle.location.local_frame.down
    location['local_frame'] = local_frame
    attributes['location'] = location

    attitude = json.loads("{}")
    attitude['pitch'] = self.vehicle.attitude.pitch
    attitude['yaw'] = self.vehicle.attitude.yaw
    attitude['roll'] = self.vehicle.attitude.roll
    attributes['attitude'] = attitude

    gps = json.loads("{}")
    gps['fix'] = self.vehicle.gps_0.fix_type
    gps['num_sat'] = self.vehicle.gps_0.satellites_visible
    attributes['gps_0'] = gps

    gimbal = json.loads("{}")
    gimbal['pitch'] = self.vehicle.gimbal.pitch
    gimbal['yaw'] = self.vehicle.gimbal.yaw
    gimbal['roll'] = self.vehicle.gimbal.roll
    attributes['gimbal'] = gimbal

    battery = json.loads("{}")
    battery['voltage'] = self.vehicle.battery.voltage
    battery['current'] = self.vehicle.battery.current
    battery['level'] = self.vehicle.battery.level
    attributes['battery'] = battery

    rangefinder = json.loads("{}")
    rangefinder['distance'] = self.vehicle.rangefinder.distance
    rangefinder['voltage'] = self.vehicle.rangefinder.voltage
    attributes['rangefinder'] = rangefinder

    channels = json.loads("{}")
    channels['channels'] = self.vehicle.channels
    channels['rssi'] = self.vehicle.channels.rssi
    attributes['channels'] = channels

    radio_status = json.loads("{}")
    radio_status['rssi'] = self.vehicle.radio_status.rssi
    radio_status['remrssi'] = self.vehicle.radio_status.remrssi
    radio_status['txbuf'] = self.vehicle.radio_status.txbuf
    radio_status['noise'] = self.vehicle.radio_status.noise
    radio_status['remnoise'] = self.vehicle.radio_status.remnoise
    radio_status['rxerrors'] = self.vehicle.radio_status.rxerrors
    radio_status['fixed'] = self.vehicle.radio_status.fixed
    attributes['radio_status'] = radio_status

    return json.dumps(attributes)

  def attributes(self):
    # Get some vehicle attributes (state)
    print "Get some vehicle attribute values:"
    print "Autopilot Firmware version:            %s" % self.vehicle.version
    print "Autopilot capabilities (supports ftp): %s" % self.vehicle.capabilities.ftp
    print "Global Location:                       %s" % self.vehicle.location.global_frame
    print "Global Location (relative altitude):   %s" % self.vehicle.location.global_relative_frame
    print "Local Location:                        %s" % self.vehicle.location.local_frame    #NED
    print "Attitude:                              %s" % self.vehicle.attitude
    print "Velocity:                              %s" % self.vehicle.velocity
    print "GPS:                                   %s" % self.vehicle.gps_0
    print "Groundspeed:                           %s" % self.vehicle.groundspeed
    print "Airspeed:                              %s" % self.vehicle.airspeed
    print "Gimbal status:                         %s" % self.vehicle.gimbal
    print "Battery:                               %s" % self.vehicle.battery
    print "EKF OK?:                               %s" % self.vehicle.ekf_ok
    print "Last Heartbeat:                        %s" % self.vehicle.last_heartbeat
    print "Rangefinder:                           %s" % self.vehicle.rangefinder
    print "Rangefinder distance:                  %s" % self.vehicle.rangefinder.distance
    print "Rangefinder voltage:                   %s" % self.vehicle.rangefinder.voltage
    print "Heading:                               %s" % self.vehicle.heading
    print "Is Armable?:                           %s" % self.vehicle.is_armable
    print "System status:                         %s" % self.vehicle.system_status.state
    print "Mode:                                  %s" % self.vehicle.mode.name    # settable
    print "Armed:                                 %s" % self.vehicle.armed    # settable

  def arm(self):
    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not self.vehicle.is_armable:
        print "Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    self.vehicle.mode    = VehicleMode("GUIDED")
    self.vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not self.vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)
    return "ARMED"

  def disarm(self):
    print "Unarming motors"
    self.vehicle.armed = False
    while self.vehicle.armed:
      print "Waiting for not armed...."
      time.sleep(1)
    return "DISARMED"

  def takeoff(self, aTargetAltitude=20):
    self.arm()

    print "Taking off!"
    self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    """
    while True:
        print " Altitude: ", self.vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if self.vehicle.location.global_relative_frame.alt >= (aTargetAltitude*0.95):
            print "Reached target altitude"
            break
        time.sleep(1)
    """
    return "ASCENDING"


  def land(self):
    print "Landing!"
    self.vehicle.mode = VehicleMode("LAND")

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    """
    while True:
        print " Altitude: ", self.vehicle.location.global_relative_frame.alt
        #Break and return from function just below target altitude.
        if self.vehicle.location.global_relative_frame.alt <= 0.0:
            print "Landed"
            break
        time.sleep(1)

    self.disarm()
    """
    return "DESCENDING"

  def fly_to(self):
    point1 = LocationGlobalRelative(-35.359980, 149.165014, 20)
    self.vehicle.airspeed = 15

    print "Fly to: %s" % point1
    self.vehicle.simple_goto(point1)
    print "FLYING"

  def return_to_launch(self):
    self.vehicle.mode = VehicleMode("RTL")
    return "RTL, INITIATED"

  def condition_yaw(self, heading, relative=False):
    if relative:
      is_relative=1 #yaw relative to direction of travel
    else:
      is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = self.vehicle.message_factory.command_long_encode(
          0, 0,    # target system, target component
          mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
          0, #confirmation
          heading,    # param 1, yaw in degrees
          0,          # param 2, yaw speed deg/s
          1,          # param 3, direction -1 ccw, 1 cw
          is_relative, # param 4, relative offset 1, absolute angle 0
          0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    self.vehicle.send_mavlink(msg)
    return "TURNING"

from simulator import Simulator
if __name__ == "__main__":
  d = Drone()
  d.config(config)
  print d
  d.connect()
  print d.get_attributes()

  #d.takeoff()
  #d.land()

  d.disconnect()
