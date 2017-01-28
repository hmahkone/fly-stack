""" Drone config """
USE_SIMULATOR=True
IDENTIFIER="XXXX"
BAUD_RATE=115200
CONNECTION_STRING="tcp:127.0.0.1:5760"

WTF_CSRF_ENABLED = True
SECRET_KEY = 'you-will-never-guess'

""" Database configs """
import os
basedir = os.path.abspath(os.path.dirname(__file__))

SQLALCHEMY_DATABASE_URI = 'sqlite:///' + os.path.join(basedir, 'gui.db')
SQLALCHEMY_MIGRATE_REPO = os.path.join(basedir, 'db_repository')
