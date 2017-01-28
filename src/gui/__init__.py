from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from flask_login import LoginManager
from api.drone import Drone
from api.simulator import Simulator

import config
drone = Drone()
drone.config(config)

sitl = None
if sitl == None and config.USE_SIMULATOR:
  sitl = Simulator()
else:
  print "Simulator already running"

gui = Flask(__name__)
gui.config.from_object('config')
db = SQLAlchemy(gui)
lm = LoginManager()
lm.init_app(gui)
lm.login_view = 'login'

from gui import views, models


