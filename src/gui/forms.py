from flask_wtf import Form
from wtforms import StringField, BooleanField
from wtforms.validators import DataRequired

class LoginForm(Form):
  email = StringField('email', validators=[DataRequired()])
  password = StringField('password', validators=[DataRequired()])
  remember_me = BooleanField('remember_me', default=False)

class NewUser(Form):
  name = StringField('email', validators=[DataRequired()])
  email= StringField('email', validators=[DataRequired()])
  password1 = StringField('email', validators=[DataRequired()])
  password2 = StringField('email', validators=[DataRequired()])