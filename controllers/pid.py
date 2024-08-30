from . import BaseController
import numpy as np

class Controller(BaseController):
  """
  A simple PID controller
  """
  def __init__(self,):
    self.p = 0.3
    self.i = 0.05
    self.d = -0.1
    self.error_integral = 0
    self.prev_error = 0

  def update(self, target_lataccel, current_lataccel, stat, future_plan):
      error = (target_lataccel - current_lataccel)
      self.error_integral += error
      error_diff = error - self.prev_error
      self.prev_error = error
      return self.p * error + self.i * self.error_integral + self.d * error_diff


class CustomFeedbackController(BaseController):
  """
  A simple PID controller
  """
  def __init__(self, kp=0.0, ki=0.0, kd=0.0):
    self.p = kp
    self.i = ki
    self.d = kd
    self.error_integral = 0
    self.prev_error = 0
    self.steer_hist = []
    self.p_err_hist = []
    self.i_err_hist = []
    self.d_err_hist = []

  def update(self, target_lataccel, current_lataccel, stat, future_plan):
      error = (target_lataccel - current_lataccel)
      self.error_integral += error
      error_diff = error - self.prev_error
      self.prev_error = error
      self.p_err_hist.append(self.p*error)
      self.i_err_hist.append(self.i*self.error_integral)
      self.d_err_hist.append(self.d*error_diff)
      steer = self.p * error + self.i * self.error_integral + self.d * error_diff
      self.steer_hist.append(steer)
      return steer
