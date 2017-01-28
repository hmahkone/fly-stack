#!/usr/bin/python
import dronekit_sitl
import os, signal, time

sim = None 

class Simulator():
  sitl = None
  def __init__(self):
    self.sitl = dronekit_sitl.start_default()

  def connection_string(self):
    return self.sitl.connection_string()

  def stop(self):
    self.sitl.stop()

def signal_handler(signum, frame):
  print "Stopping the simulator."
  if sim:
    sim.stop()
  exit(0)

if __name__ == "__main__":
  signal.signal(signal.SIGINT, signal_handler)
  sim = Simulator()
  while True:
    time.sleep(1)

