from selfdrive.car import limit_steer_rate
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_lkas12, \
                                             create_1191, create_1156, \
                                             learn_checksum, create_mdps12, create_clu11

from selfdrive.car.hyundai.values import Buttons
from selfdrive.can.packer import CANPacker
import zmq
from selfdrive.services import service_list
import selfdrive.messaging as messaging
from selfdrive.config import Conversions as CV
from common.params import Params
from selfdrive.swaglog import cloudlog
import time



# Steer torque limits

class SteerLimitParams:
  STEER_MAX = 255   # >255 results in frozen torque, >409 results in no torque
  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 5
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  DIVIDER = 2.0     # Must be > 1.0

class CarController(object):
  def __init__(self, dbc_name, car_fingerprint):
    self.apply_steer_last = 0
    self.car_fingerprint = car_fingerprint

    self.lkas11_cnt = 0
    self.clu11_cnt = 0
    self.mdps12_cnt = 0
    self.cnt = 0
    self.last_resume_cnt = 0

    self.map_speed = 0
    self.map_data_sock = messaging.sub_sock(service_list['liveMapData'].port)
    self.params = Params()
    self.speed_conv = 3.6
    self.speed_offset = 1.03      # Multiplier for cruise speed vs speed limit  TODO: Add to UI
    self.speed_enable = True      # Enable Auto Speed Set                       TODO: Add to UI
    self.speed_adjusted = False

    self.checksum = "NONE"
    self.checksum_learn_cnt = 0

    self.turning_signal_timer = 0
    self.camera_disconnected = False
    self.checksum_found = False

    self.packer = CANPacker(dbc_name)
    
    self.min_steer_start_time = int(time.time()*1000.0)
    self.duty_cycle_steering = False

  def update(self, enabled, CS, actuators, pcm_cancel_cmd, hud_alert):
    ### Error State Resets ###
    disable_steer = False
    can_sends = []

    ### Learn Checksum ###

    if not self.checksum_found:
      # Learn Checksum from the Camera
      if self.checksum == "NONE":
        self.checksum = learn_checksum(self.packer, CS.lkas11)
        if self.checksum == "NONE" and self.checksum_learn_cnt < 50:
          self.checksum_learn_cnt += 1
          return
        else:
          cloudlog.info("Discovered Checksum %s" % self.checksum)
          self.checksum_found = True

      # If MDPS is faulted from bad checksum, then cycle through all Checksums until 1 works
      if CS.steer_error == 1:
        self.camera_disconnected = True
        cloudlog.warning("Camera Not Detected: Brute Forcing Checksums")
        if self.checksum_learn_cnt > 300:
          self.checksum_learn_cnt = 50
          if self.checksum == "NONE":
            cloudlog.info("Testing 6B Checksum")
            self.checksum = "6B"
          elif self.checksum == "6B":
            cloudlog.info("Testing 7B Checksum")
            self.checksum = "7B"
          elif self.checksum == "7B":
            cloudlog.info("Testing CRC8 Checksum")
            self.checksum = "crc8"
          else:
            self.checksum = "NONE"
            return
        else:
          self.checksum_learn_cnt += 1
      else:
        cloudlog.info("Discovered Checksum %s" % self.checksum)
        self.checksum_found = True

    ### Minimum Steer Speed ###

    # Apply Usage of Minimum Steer Speed
    #if CS.low_speed_alert:
    #  disable_steer = True

    if CS.v_ego_raw < 14.7: #CS.min_steer_speed: #we are below minimum speed for normal steering
      if CS.mdps12_flt == 1: #we faulted the mdps, wait for it to unfault
        disable_steer = True
      else:
         # mdps takes 0.02 seconds to respond to steer request and set steer_state
         # this toggles the request every 0.02 seconds plus small unknown error to read or set state
         # state must be toggled no later than 0.04 seconds
         disable_steer = CS.steer_state

    ### Turning Indicators ###
    if (CS.left_blinker_on == 1 or CS.right_blinker_on == 1):
      self.turning_signal_timer = 100 # Disable for 1.0 Seconds after blinker turned off

    if self.turning_signal_timer > 0:
      disable_steer = True
      self.turning_signal_timer -= 1

    ### Steering Torque ###
    apply_steer = actuators.steer * SteerLimitParams.STEER_MAX
    apply_steer = limit_steer_rate(apply_steer, self.apply_steer_last, CS.steer_torque_driver, SteerLimitParams)

    if not enabled or disable_steer:
      apply_steer = 0
      steer_req = 0
    else:
      steer_req = 1

    self.apply_steer_last = apply_steer

    '''
    ### Auto Speed Limit ###

    # Read Speed Limit and define if adjustment needed
    if (self.cnt % 50) == 0 and self.speed_enable:
      if not (enabled and CS.acc_active):
        self.speed_adjusted = False
      map_data = messaging.recv_one_or_none(self.map_data_sock)
      if map_data is not None:
        if bool(self.params.get("IsMetric")):
          self.speed_conv = CV.MS_TO_KPH
        else:
          self.speed_conv = CV.MS_TO_MPH

        if map_data.liveMapData.speedLimitValid:
          last_speed = self.map_speed
          v_speed = int(map_data.liveMapData.speedLimit * self.speed_offset)
          self.map_speed = v_speed * self.speed_conv
          if last_speed != self.map_speed:
            self.speed_adjusted = False
        else:
          self.map_speed = 0
          self.speed_adjusted = True
    else:
      self.map_speed = 0
      self.speed_adjusted = True

    # Spam buttons for Speed Adjustment
    if CS.acc_active and not self.speed_adjusted and self.map_speed > (8.5 * self.speed_conv) and (self.cnt % 9 == 0 or self.cnt % 9 == 1):
      if (CS.cruise_set_speed * self.speed_conv) > (self.map_speed * 1.005):
        can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.SET_DECEL, (1 if self.cnt % 9 == 1 else 0)))
      elif (CS.cruise_set_speed * self.speed_conv) < (self.map_speed / 1.005):
        can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL, (1 if self.cnt % 9 == 1 else 0)))
      else:
        self.speed_adjusted = True

    # Cancel Adjustment on Pedal
    if CS.pedal_gas:
      self.speed_adjusted = True

    '''

    ### Generate CAN Messages ###

    self.lkas11_cnt = self.cnt % 0x10
#   self.clu11_cnt = self.cnt % 0x10
    self.mdps12_cnt = self.cnt % 0x100

    if self.camera_disconnected:
      if (self.cnt % 10) == 0:
        can_sends.append(create_lkas12())
      if (self.cnt % 50) == 0:
        can_sends.append(create_1191())
      if (self.cnt % 7) == 0:
        can_sends.append(create_1156())

    can_sends.append(create_lkas11(self.packer, self.car_fingerprint, apply_steer, steer_req, self.lkas11_cnt,
                                   enabled, CS.lkas11, hud_alert, (not self.camera_disconnected), self.checksum))

    if not self.camera_disconnected:
      can_sends.append(create_mdps12(self.packer, self.car_fingerprint, self.mdps12_cnt, CS.mdps12, CS.lkas11, \
                                    self.checksum))

#    if pcm_cancel_cmd:
#      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.CANCEL, 0))
    if CS.stopped and (self.cnt - self.last_resume_cnt) > 20:
      if (self.cnt - self.last_resume_cnt) > 20:
        self.last_resume_cnt = self.cnt
      can_sends.append(create_clu11(self.packer, CS.clu11, Buttons.RES_ACCEL, self.clu11_cnt))

    self.cnt += 1

    return can_sends
