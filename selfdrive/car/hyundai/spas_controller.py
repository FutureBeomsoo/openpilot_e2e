import crcmod
from cereal import car
from common.params import Params
from common.numpy_fast import clip, interp
from common.conversions import Conversions as CV
from selfdrive.car.hyundai.values import CHECKSUM

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)

EventName = car.CarEvent.EventName

# SPAS Constants
STEER_ANG_MAX = 360  # SPAS Max Angle (degrees)
ANGLE_DELTA_BP = [0., 10., 20.]
ANGLE_DELTA_V = [1.19, 1.14, 1.09]    # windup limit
ANGLE_DELTA_VU = [1.29, 1.19, 1.14]   # unwind limit
SPAS_OVERRIDE_TQ = 290  # driver override torque threshold (unit: torque / 100 = Nm)
SPAS_SWITCH_SPEED = 30 * CV.MPH_TO_MS  # speed threshold for dynamic SPAS/LKAS switching
STEER_MAX_OFFSET = 105  # torque offset for dynamic SPAS engagement


class SpasController:
  def __init__(self, car_fingerprint):
    self.car_fingerprint = car_fingerprint
    self.last_apply_angle = 0.0
    self.en_spas = 2
    self.mdps11_stat_last = 0
    self.spas_active = False
    self.ratelimit = 2.3
    self.rate = 0
    self.lastSteeringAngleDeg = 0
    self.cut_timer = 0
    self.steer_temp_unavailable = False
    self.debug = Params().get_bool('SpasDebug')

  def inject_events(self, events):
    if self.steer_temp_unavailable:
      events.add(EventName.steerTempUnavailable)

  def update(self, CC, CS, actuators, frame, steer_max, packer, apply_steer, can_sends):
    """Main SPAS controller update - called every frame when SPAS is enabled."""
    # Track steering wheel rate
    self.rate = abs(CS.out.steeringAngleDeg - self.lastSteeringAngleDeg)
    apply_angle = clip(actuators.steeringAngleDeg, -STEER_ANG_MAX, STEER_ANG_MAX)
    apply_diff = abs(apply_angle - CS.out.steeringAngleDeg)

    # Determine if SPAS should be active
    spas_active = (
      CC.latActive and
      CS.out.vEgo < 26.82 and
      (CS.out.vEgo < SPAS_SWITCH_SPEED or
       (apply_diff > 3.2 and not CS.out.steeringPressed) or
       (abs(apply_angle) > 3.0 and self.spas_active) or
       (steer_max - STEER_MAX_OFFSET < abs(apply_steer)))
    )

    # Rate limiting (runs at SPAS11 message rate = 50Hz, every 2 frames)
    if (frame % 2) == 0:
      if CS.spas_mdps11_stat == 5 and apply_diff > 1.75:
        # Engage rate: ramp up when angle difference is large
        self.ratelimit += 0.03
        rate_limit = max(self.ratelimit, 10)
        apply_angle = clip(apply_angle,
                           CS.out.steeringAngleDeg - rate_limit,
                           CS.out.steeringAngleDeg + rate_limit)
      elif CS.spas_mdps11_stat == 5:
        # Normal operation rate limiter
        self.ratelimit = 2.3
        if self.last_apply_angle * apply_angle > 0. and abs(apply_angle) > abs(self.last_apply_angle):
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_V)
        else:
          rate_limit = interp(CS.out.vEgo, ANGLE_DELTA_BP, ANGLE_DELTA_VU)
        apply_angle = clip(apply_angle,
                           self.last_apply_angle - rate_limit,
                           self.last_apply_angle + rate_limit)
      else:
        apply_angle = CS.spas_mdps11_strang

    # Driver override detection
    if CS.spas_steering_pressed or self.rate > 1.2:
      self.cut_timer = 0
      spas_active = False

    if CS.spas_steering_pressed or self.cut_timer <= 100:
      spas_active = False
      self.cut_timer += 1

    self.last_apply_angle = apply_angle

    # SPAS State Machine
    # Speed spoofing when MDPS is in states 3, 4, or 5
    spas_active_stat = spas_active and CS.spas_mdps11_stat in (3, 4, 5)

    # EMS spoofing on Bus 1 (MDPS CAN) - for EV: E_EMS11
    can_sends.append(_create_eems11(packer, CS.spas_eems11, spas_active_stat))

    # ELECT_GEAR spoofing on Bus 1 - keep gear shifter, zero the rest
    can_sends.append(_create_elect_gear_spoof(packer, CS.spas_elect_gear_shifter, spas_active_stat))

    if (frame % 2) == 0:
      # SPAS State Machine transitions
      if CS.spas_mdps11_stat == 7 and self.mdps11_stat_last != 7:
        self.en_spas = 7  # Acknowledge MDPS state 7

      if CS.spas_mdps11_stat == 7 and self.mdps11_stat_last == 7:
        self.en_spas = 3  # Ready for next steer

      if CS.spas_mdps11_stat == 2 and spas_active:
        self.en_spas = 3  # Ready to Assist

      if CS.spas_mdps11_stat == 3 and spas_active:
        self.en_spas = 4  # Handshake

      if CS.spas_mdps11_stat == 4:
        self.en_spas = 5  # Request steering

      if CS.spas_mdps11_stat == 5 and not spas_active:
        self.en_spas = 7  # Cancel SPAS

      if CS.spas_mdps11_stat == 6:
        self.en_spas = 2  # Failed, reset

      if CS.spas_mdps11_stat == 8:
        self.en_spas = 2  # Failed to get ready, reset

      # Monitor MDPS error states
      self.steer_temp_unavailable = CS.spas_mdps11_stat in (6, 8)

      if not spas_active:
        apply_angle = CS.spas_mdps11_strang

      # Send SPAS11 message
      can_sends.append(_create_spas11(packer, self.car_fingerprint, frame // 2,
                                      self.en_spas, apply_angle, 1))  # bus=1 (MDPS CAN)

    # Send SPAS12 at 20Hz (every 5 frames)
    if (frame % 5) == 0:
      can_sends.append(_create_spas12(1))  # bus=1 (MDPS CAN)

    if self.debug:
      print(f"SPAS | MDPS:{CS.spas_mdps11_stat} OP:{self.en_spas} active:{spas_active} angle:{apply_angle:.1f} drv_tq:{CS.out.steeringTorque:.0f}")

    self.mdps11_stat_last = CS.spas_mdps11_stat
    self.spas_active = spas_active
    self.lastSteeringAngleDeg = CS.out.steeringAngleDeg

    return spas_active


def _create_spas11(packer, car_fingerprint, frame, en_spas, apply_steer, bus):
  values = {
    "CF_Spas_Stat": en_spas,
    "CF_Spas_TestMode": 0,
    "CR_Spas_StrAngCmd": apply_steer,
    "CF_Spas_BeepAlarm": 0,
    "CF_Spas_Mode_Seq": 1,  # non-legacy
    "CF_Spas_AliveCnt": frame % 0x200,
    "CF_Spas_Chksum": 0,
    "CF_Spas_PasVol": 0,
  }
  dat = packer.make_can_msg("SPAS11", 0, values)[2]
  if car_fingerprint in CHECKSUM["crc8"]:
    dat = dat[:6]
    values["CF_Spas_Chksum"] = hyundai_checksum(dat)
  else:
    values["CF_Spas_Chksum"] = sum(dat[:6]) % 256
  return packer.make_can_msg("SPAS11", bus, values)


def _create_spas12(bus):
  return [1268, 0, b"\x00\x00\x00\x00\x00\x00\x00\x00", bus]


def _create_eems11(packer, eems11_values, spas_active):
  if spas_active:
    values = {
      "Brake_Pedal_Pos": 0,
      "IG_Reactive_Stat": 0,
      "Gear_Change": 0,
      "Cruise_Limit_Status": 0,
      "Cruise_Limit_Target": 0,
      "Accel_Pedal_Pos": 0,
      "CR_Vcu_AccPedDep_Pos": 0,
    }
  else:
    values = dict(eems11_values)
  return packer.make_can_msg("E_EMS11", 1, values)


def _create_elect_gear_spoof(packer, gear_shifter, spas_active):
  if spas_active:
    values = {"Elect_Gear_Shifter": gear_shifter}
  else:
    values = {"Elect_Gear_Shifter": gear_shifter}
  return packer.make_can_msg("ELECT_GEAR", 1, values)
