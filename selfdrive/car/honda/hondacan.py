import struct

import common.numpy_fast as np
from selfdrive.config import Conversions as CV
from selfdrive.car.honda.values import CAR, HONDA_BOSCH, VEHICLE_STATE_MSG
from numpy import array as nparray

# *** Honda specific ***
def can_cksum(mm):
  s = 0
  for c in mm:
    c = ord(c)
    s += (c>>4)
    s += c & 0xF
  s = 8-s
  s %= 0x10
  return s


def fix(msg, addr):
  msg2 = msg[0:-1] + chr(ord(msg[-1]) | can_cksum(struct.pack("I", addr)+msg))
  return msg2


def make_can_msg(addr, dat, idx, alt):
  if idx is not None:
    dat += chr(idx << 4)
    dat = fix(dat, addr)
  return [addr, 0, dat, alt]


def create_brake_command(packer, apply_brake, pcm_override, pcm_cancel_cmd, chime, fcw, idx):
  """Creates a CAN message for the Honda DBC BRAKE_COMMAND."""
  pump_on = apply_brake > 0
  brakelights = apply_brake > 0
  brake_rq = apply_brake > 0
  pcm_fault_cmd = False

  values = {
    "COMPUTER_BRAKE": apply_brake,
    # "BRAKE_PUMP_REQUEST": pump_on,
    "CRUISE_OVERRIDE": pcm_override,
    "CRUISE_FAULT_CMD": pcm_fault_cmd,
    "CRUISE_CANCEL_CMD": pcm_cancel_cmd,
    "COMPUTER_BRAKE_REQUEST": brake_rq,
    "SET_ME_0X80": 0x80,
    "BRAKE_LIGHTS": brakelights,
    "CHIME": chime,
    "FCW": fcw, #tsedited # TODO: Why are there two bits for fcw? According to dbc file the first bit should also work
  }
  return packer.make_can_msg("BRAKE_COMMAND", 0, values, idx)


def create_gas_command(packer, gas_amount, idx):
  """Creates a CAN message for the Honda DBC GAS_COMMAND."""
  enable = gas_amount > 0.001

  values = {"ENABLE": enable}

  if enable:
    values["GAS_COMMAND"] = gas_amount * 255.
    values["GAS_COMMAND2"] = gas_amount * 255.

  return packer.make_can_msg("GAS_COMMAND", 0, values, idx)
# below is if panda is sending the LIN over UART... it does not do it timely... so
#alter the signal for arduino LIN2LIN repeater.. new signal is 2 bytes CCS#####  CC = counter  A=state(lkas_active)
#                                          byte #1 CCA0BBBB  #2 CCALLLLL        ##### = big steer byte 1, little on byte 2
def create_steering_control_serial(frame, counter, big_steer, lkas_on, little_steer, lkas_off):
  datapack = nparray([0x00, 0x00, 0x00, 0x00], dtype='uint8')
# 128  64  32  16  8  4  2  1
  datapack[0] = (big_steer + counter)
  datapack[1] = (little_steer + lkas_on + 128)
  datapack[2] =  (lkas_off + 128)
  datapack[3] = create_steering_control_serial_checksum(datapack[0],datapack[1],datapack[2])
  if (frame % 300) == 0:
    print datapack
  return datapack

def create_steering_control_serial_lin2linrepeater(big_steer, lkas_on, little_steer):
  datapack = [0x00,0x00,0x00,0x00]
# 128  64  32  16  8  4  2  1
  thislkas = lkas_on << 5
  datapack[0] = big_steer | 0x40 | thislkas
  datapack[1] = little_steer | 0x80 | thislkas
  if (frame % 300) == 0:
    print datapack
  return bytearray(datapack)

def create_steering_control_serial_checksum(a,b,c):
  total = a + b + c
  total = total % 512
  total = 512 - total
  return total % 256

def create_steering_control_serial_candata(packer, counter, big_steer, lkas_on, little_steer, lkas_off, apply_steer, actuator_steer):
  values = {
      "BIG_STEER": big_steer,
      "SERIAL_COUNTER": counter,
      "LITTLE_STEER": little_steer,
      "LKAS_ON": lkas_on,
      "SET_1": 1,
      "LKAS_OFF": lkas_off,
      "SET_1_1": 1,
      "SERIAL_CHECKSUM": chksm,
  }
  return packer.make_can_msg("LKAS_SERIAL", 2, values)
  return 1


#create_steering_control_serial_candata(self.packer, self.counter, big_steer, lkas_on, little_steer, lkas_off, chksm, apply_steer, int(clip(actuators.steer * 100,0,100))))
def create_steering_control(packer, apply_steer, lkas_active, car_fingerprint, idx):
  """Creates a CAN message for the Honda DBC STEERING_CONTROL."""
  values = {
    "STEER_TORQUE": apply_steer if lkas_active else 0,
    "STEER_TORQUE_REQUEST": lkas_active,
  }
  # Set bus 2 for accord and new crv.
  bus = 2 if car_fingerprint in HONDA_BOSCH else 0
  return packer.make_can_msg("STEERING_CONTROL", bus, values, idx)


def create_ui_commands(packer, pcm_speed, hud, car_fingerprint, idx, dashed_lanes, signal_changed):
  """Creates an iterable of CAN messages for the UIs."""
  commands = []
  bus = 0

  # Bosch sends commands to bus 2.
  if car_fingerprint in HONDA_BOSCH:
    bus = 2
  else:
    acc_hud_values = {
      'PCM_SPEED': pcm_speed * CV.MS_TO_MPH,
      'PCM_GAS': hud.pcm_accel,
      'CRUISE_SPEED': hud.v_cruise,
      'ENABLE_MINI_CAR': hud.mini_car,
      'HUD_LEAD': hud.car,
      }
    if car_fingerprint in (CAR.ACCORD_2016):
      bus = 0
#    commands.append(packer.make_can_msg("ACC_HUD", bus, acc_hud_values, idx))



  lkas_hud_values = {
    'STEERING_REQUIRED': hud.steer_required,
    'SOLID_LANES': hud.lanes,
    'BEEP': hud.beep,
    'DASHED_LANES': dashed_lanes,
    'SIGNAL_CHANGED': signal_changed
  }
  commands.append(packer.make_can_msg('LKAS_HUD', 0, lkas_hud_values, idx))

  if car_fingerprint in (CAR.CIVIC, CAR.ODYSSEY):
    commands.append(packer.make_can_msg('HIGHBEAM_CONTROL', 0, {'HIGHBEAMS_ON': False}, idx))

    radar_hud_values = {
      'ACC_ALERTS': hud.acc_alert,
      'LEAD_SPEED': 0x1fe,  # What are these magic values
      'LEAD_STATE': 0x7,
      'LEAD_DISTANCE': 0x1e,
    }
    commands.append(packer.make_can_msg('RADAR_HUD', 0, radar_hud_values, idx))
  return commands


def create_radar_commands(packer,v_ego, car_fingerprint, new_radar_config, idx):
  """Creates an iterable of CAN messages for the radar system."""
  commands = []
  v_ego_kph = np.clip(int(round(v_ego * CV.MS_TO_KPH)), 0, 255)
  speed = struct.pack('!B', v_ego_kph)

  x300_values = {
    'SET_ME_XF9': 0xf9,
    'VEHICLE_SPEED': v_ego_kph,
  }
  x301_values = {
    'SET_ME_0F18510': 0x0F18510,
    'SET_ME_25A0000': 0x25A0000,
  }
  commands.append(packer.make_can_msg('VEHICLE_STATE', 1, x300_values, idx))
  commands.append(packer.make_can_msg('VEHICLE_STATE2', 1, x301_values, idx))
  # commands.append(make_can_msg(0x300, msg_0x300, idx_0x300, 1))

  # commands.append(make_can_msg(0x301, msg_0x301, idx_0x301, 1))
  return commands

def spam_buttons_command(packer, button_val, idx):
  values = {
    'CRUISE_BUTTONS': button_val,
    'CRUISE_SETTING': named,
  }
  return packer.make_can_msg("SCM_BUTTONS", 0, values, idx)


  # values = {
  #   "BIG_STEER": big_steer,
  #   "SERIAL_COUNTER": counter,
  #   "LITTLE_STEER": little_steer,
  #   "LKAS_ON": lkas_on,
  #   "SET_1": 1,
  #   "LKAS_OFF": lkas_off,
  #   "SET_1_1": 1,
  #   "SERIAL_CHECKSUM": chksm,
  #   }
  # return packer.make_can_msg("LKAS_SERIAL", 2, values)
