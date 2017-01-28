#!/usr/bin/python
import os
from gui import gui
gui.secret_key = os.urandom(12)
gui.run(debug=True)