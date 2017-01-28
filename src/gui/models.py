from gui import db

class User(db.Model):
  id = db.Column(db.Integer, primary_key=True)
  email = db.Column(db.String(120), index=True, unique=True)
  name = db.Column(db.String)
  password = db.Column(db.String)
  authenticated = db.Column(db.Boolean, default=False)

  @property
  def is_authenticated(self):
    return True

  @property
  def is_active(self):
    return True

  @property
  def is_anonymous(self):
    return False

  def get_id(self):
    try:
      return unicode(self.id)  # python 2
    except NameError:
      return str(self.id)  # python 3

  def __repr__(self):
    return '<User %r>' % (self.email)
