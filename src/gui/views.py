from gui import gui, db, lm, drone
from flask import Flask, flash, redirect, render_template, request, session, url_for, abort, g, jsonify
from flask_login import login_user, logout_user, current_user, login_required
from .forms import LoginForm, NewUser
from .models import User

@lm.user_loader
def load_user(id):
  return User.query.get(int(id))

@gui.before_request
def before_request():
  g.user = current_user
  g.drone = drone

@gui.route('/attributes')
#@login_required
def attributes():
  return g.drone.get_attributes()

@gui.route('/connect')
#@login_required
def connect():
  print "connect"
  return g.drone.connect()

@gui.route('/disconnect')
#@login_required
def disconnect():
  print "disconnect"
  return g.drone.disconnect()

@gui.route('/takeoff')
#@login_required
def takeoff():
  print "takeoff"
  return g.drone.takeoff()

@gui.route('/land')
#@login_required
def land():
  print "land"
  return g.drone.land()

@gui.route('/fly')
#@login_required
def fly():
  print "fly to"
  return g.drone.fly_to()

@gui.route('/rtl')
#@login_required
def rtl():
  print "RTL"
  return g.drone.return_to_launch()

@gui.route('/heading/<int:heading>', methods=['POST'])
def heading(heading):
  print "Heading: %d" % heading
  return g.drone.condition_yaw(heading)

@gui.route('/')
@gui.route('/index')
#@login_required
def index():
  user = g.user
  return render_template('index.html',
                         title='Home',
                         user=user,
                         drone=drone)

@gui.route('/register', methods=['GET', 'POST'])
def register():
  form = NewUser()
  if form.validate_on_submit():
    print "Name: %s" % form.name.data
    print "Email: %s" % form.email.data
    print "Password1: %s" % form.password1.data
    print "Password2: %s" % form.password2.data
    user = User.query.filter_by(email=form.email.data).first()
    if user is None:
      user = User(name=form.name.data, email=form.email.data,
                    password=form.password1.data)
      print "Adding a user"
      db.session.add(user)
      db.session.commit()
      return redirect(url_for("login"))
    else:
      print "User exists"
      return render_template("register.html", form=form)
  else:
    return render_template("register.html", form=form)

@gui.route('/map', methods=['GET'])
def map():
  return render_template("map.html", drone=drone)

@gui.route('/login', methods=['GET', 'POST'])
def login():
  if g.user is not None and g.user.is_authenticated:
    return redirect(url_for('index'))
  form = LoginForm()
  print "Email: %s" % form.email.data
  print "Password: %s" % form.password.data
  if form.validate_on_submit():
    print "APINA 1"
    user = User.query.filter_by(email=form.email.data).first()
    if user:
      print "Name: %s" % user.name
      print "Email: %s" % user.email
      print "Pw: %s" % user.password
      if user.password == form.password.data:
        user.authenticated = True
        db.session.add(user)
        db.session.commit()
        login_user(user, remember=True)
        return redirect(url_for("index"))
      else:
        print "Wrong password"
    else:
      print "No user found for %s" % form.email.data
  else:
    print "Form invalid"
  return render_template("login.html", form=form)

@gui.route('/logout')
def logout():
    g.drone.disconnect()
    logout_user()
    return redirect(url_for('index'))
