from dronekit import connect, VehicleMode
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
