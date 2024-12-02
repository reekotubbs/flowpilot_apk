#test workflow push
from dataclasses import dataclass, field
from enum import Enum, IntFlag

from cereal import car
from common.numpy_fast import interp
from selfdrive.car import dbc_dict, PlatformConfig, DbcDict, Platforms, CarSpecs
from selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarInfo, CarParts, Column
from selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_MAX = 300  # GM limit is 3Nm. Used by carcontroller to generate LKA output
  STEER_STEP = 3  # Active control frames per command (~33hz)
  INACTIVE_STEER_STEP = 10  # Inactive control frames per command (10hz)
  STEER_DELTA_UP = 10  # Delta rates require review due to observed EPS weakness
  STEER_DELTA_DOWN = 15
  STEER_DRIVER_ALLOWANCE = 65
  STEER_DRIVER_MULTIPLIER = 4
  STEER_DRIVER_FACTOR = 100
  NEAR_STOP_BRAKE_PHASE = 0.25  # m/s
  SNG_INTERCEPTOR_GAS = 18. / 255.
  SNG_TIME = 30  # frames until the above is reached

  # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
  ADAS_KEEPALIVE_STEP = 100
  CAMERA_KEEPALIVE_STEP = 100

  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  def __init__(self, CP):
    # Gas/brake lookups
    self.ZERO_GAS = 2048  # Coasting
    self.MAX_BRAKE = 400  # ~ -4.0 m/s^2 with regen

    if CP.carFingerprint in CAMERA_ACC_CAR and CP.carFingerprint not in CC_ONLY_CAR:
      self.MAX_GAS = 3400
      self.MAX_ACC_REGEN = 1514
      self.INACTIVE_REGEN = 1554
      # Camera ACC vehicles have no regen while enabled.
      # Camera transitions to MAX_ACC_REGEN from ZERO_GAS and uses friction brakes instantly
      max_regen_acceleration = 0.

    elif CP.carFingerprint in SDGM_CAR:
      self.MAX_GAS = 3400
      self.MAX_ACC_REGEN = 1514
      self.INACTIVE_REGEN = 1554
      max_regen_acceleration = 0.

    else:
      self.MAX_GAS = 3072  # Safety limit, not ACC max. Stock ACC >4096 from standstill.
      self.MAX_ACC_REGEN = 1404  # Max ACC regen is slightly less than max paddle regen
      self.INACTIVE_REGEN = 1404
      # ICE has much less engine braking force compared to regen in EVs,
      # lower threshold removes some braking deadzone
      max_regen_acceleration = -1. if CP.carFingerprint in EV_CAR else -0.1

    self.GAS_LOOKUP_BP = [max_regen_acceleration, 0., self.ACCEL_MAX]
    self.GAS_LOOKUP_V = [self.MAX_ACC_REGEN, self.ZERO_GAS, self.MAX_GAS]

    self.BRAKE_LOOKUP_BP = [self.ACCEL_MIN, max_regen_acceleration]
    self.BRAKE_LOOKUP_V = [self.MAX_BRAKE, 0.]

  # determined by letting Volt regen to a stop in L gear from 89mph,
  # and by letting off gas and allowing car to creep, for determining
  # the positive threshold values at very low speed
  EV_GAS_BRAKE_THRESHOLD_BP = [1.29, 1.52, 1.55, 1.6, 1.7, 1.8, 2.0, 2.2, 2.5, 5.52, 9.6, 20.5, 23.5, 35.0] # [m/s]
  EV_GAS_BRAKE_THRESHOLD_V = [0.0, -0.14, -0.16, -0.18, -0.215, -0.255, -0.32, -0.41, -0.5, -0.72, -0.895, -1.125, -1.145, -1.16] # [m/s^s]

  def update_ev_gas_brake_threshold(self, v_ego):
    gas_brake_threshold = interp(v_ego, self.EV_GAS_BRAKE_THRESHOLD_BP, self.EV_GAS_BRAKE_THRESHOLD_V)
    self.EV_GAS_LOOKUP_BP = [gas_brake_threshold, max(0., gas_brake_threshold), self.ACCEL_MAX]
    self.EV_BRAKE_LOOKUP_BP = [self.ACCEL_MIN, gas_brake_threshold]


class Footnote(Enum):
  OBD_II = CarFootnote(
    'Requires a <a href="https://github.com/commaai/openpilot/wiki/GM#hardware" target="_blank">community built ASCM harness</a>. ' +
    '<b><i>NOTE: disconnecting the ASCM disables Automatic Emergency Braking (AEB).</i></b>',
    Column.MODEL)


@dataclass
class GMCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control (ACC)"

  def init_make(self, CP: car.CarParams):
    if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera:
      self.car_parts = CarParts.common([CarHarness.gm])
    else:
      self.car_parts = CarParts.common([CarHarness.obd_ii])
      self.footnotes.append(Footnote.OBD_II)


@dataclass(frozen=True, kw_only=True)
class GMCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.444  # not optimized yet


@dataclass
class GMPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('gm_global_a_powertrain_generated', 'gm_global_a_object', chassis_dbc='gm_global_a_chassis'))


class CAR(Platforms):
  HOLDEN_ASTRA = GMPlatformConfig(
    "HOLDEN ASTRA RS-V BK 2017",
    GMCarInfo("Holden Astra 2017"),
    GMCarSpecs(mass=1363, wheelbase=2.662, steerRatio=15.7, centerToFrontRatio=0.4),
  )
  VOLT = GMPlatformConfig(
    "CHEVROLET VOLT PREMIER 2017",
    GMCarInfo("Chevrolet Volt 2017-18", min_enable_speed=0, video_link="https://youtu.be/QeMCN_4TFfQ"),
    GMCarSpecs(mass=1607, wheelbase=2.69, steerRatio=17.7, centerToFrontRatio=0.45, tireStiffnessFactor=0.469, minEnableSpeed=-1),
    dbc_dict=dbc_dict('gm_global_a_powertrain_volt', 'gm_global_a_object', chassis_dbc='gm_global_a_chassis')
  )
  CADILLAC_ATS = GMPlatformConfig(
    "CADILLAC ATS Premium Performance 2018",
    GMCarInfo("Cadillac ATS Premium Performance 2018"),
    GMCarSpecs(mass=1601, wheelbase=2.78, steerRatio=15.3),
  )
  MALIBU = GMPlatformConfig(
    "CHEVROLET MALIBU PREMIER 2017",
    GMCarInfo("Chevrolet Malibu Premier 2017"),
    GMCarSpecs(mass=1496, wheelbase=2.83, steerRatio=15.8, centerToFrontRatio=0.4),
  )
  ACADIA = GMPlatformConfig(
    "GMC ACADIA DENALI 2018",
    GMCarInfo("GMC Acadia 2018", video_link="https://www.youtube.com/watch?v=0ZN6DdsBUZo"),
    GMCarSpecs(mass=1975, wheelbase=2.86, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  BUICK_LACROSSE = GMPlatformConfig(
    "BUICK LACROSSE 2017",
    GMCarInfo("Buick LaCrosse 2017-19", "Driver Confidence Package 2"),
    GMCarSpecs(mass=1712, wheelbase=2.91, steerRatio=15.8, centerToFrontRatio=0.4),
  )
  BUICK_REGAL = GMPlatformConfig(
    "BUICK REGAL ESSENCE 2018",
    GMCarInfo("Buick Regal Essence 2018"),
    GMCarSpecs(mass=1714, wheelbase=2.83, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  ESCALADE = GMPlatformConfig(
    "CADILLAC ESCALADE 2017",
    GMCarInfo("Cadillac Escalade 2017", "Driver Assist Package"),
    GMCarSpecs(mass=2564, wheelbase=2.95, steerRatio=17.3),
  )
  ESCALADE_ESV = GMPlatformConfig(
    "CADILLAC ESCALADE ESV 2016",
    GMCarInfo("Cadillac Escalade ESV 2016", "Adaptive Cruise Control (ACC) & LKAS"),
    GMCarSpecs(mass=2739, wheelbase=3.302, steerRatio=17.3, tireStiffnessFactor=1.0),
  )
  ESCALADE_ESV_2019 = GMPlatformConfig(
    "CADILLAC ESCALADE ESV 2019",
    GMCarInfo("Cadillac Escalade ESV 2019", "Adaptive Cruise Control (ACC) & LKAS"),
    ESCALADE_ESV.specs,
  )
  BOLT_EUV = GMPlatformConfig(
    "CHEVROLET BOLT EUV 2022",
    [
      GMCarInfo("Chevrolet Bolt EUV 2022-23", "Premier or Premier Redline Trim without Super Cruise Package", video_link="https://youtu.be/xvwzGMUA210"),
      GMCarInfo("Chevrolet Bolt EV 2022-23", "2LT Trim with Adaptive Cruise Control Package"),
    ],
    GMCarSpecs(mass=1669, wheelbase=2.63779, steerRatio=16.8, centerToFrontRatio=0.4, tireStiffnessFactor=1.0),
  )
  SILVERADO = GMPlatformConfig(
    "CHEVROLET SILVERADO 1500 2020",
    [
      GMCarInfo("Chevrolet Silverado 1500 2020-21", "Safety Package II"),
      GMCarInfo("GMC Sierra 1500 2020-21", "Driver Alert Package II", video_link="https://youtu.be/5HbNoBLzRwE"),
    ],
    GMCarSpecs(mass=2450, wheelbase=3.75, steerRatio=16.3, tireStiffnessFactor=1.0),
  )
  EQUINOX = GMPlatformConfig(
    "CHEVROLET EQUINOX 2019",
    GMCarInfo("Chevrolet Equinox 2019-22"),
    GMCarSpecs(mass=1588, wheelbase=2.72, steerRatio=14.4, centerToFrontRatio=0.4),
  )
  TRAILBLAZER = GMPlatformConfig(
    "CHEVROLET TRAILBLAZER 2021",
    GMCarInfo("Chevrolet Trailblazer 2021-22"),
    GMCarSpecs(mass=1345, wheelbase=2.64, steerRatio=16.8, centerToFrontRatio=0.4, tireStiffnessFactor=1.0),
  )
  # Separate car def is required when there is no ASCM
  # (for now) unless there is a way to detect it when it has been unplugged...
  VOLT_CC = GMPlatformConfig(
    "CHEVROLET VOLT NO ACC",
    VOLT.car_info,
    VOLT.specs,
  )
  BOLT_CC = GMPlatformConfig(
    "CHEVROLET BOLT EV NO ACC",
    BOLT_EUV.car_info,
    BOLT_EUV.specs,
  )
  EQUINOX_CC = GMPlatformConfig(
    "CHEVROLET EQUINOX NO ACC",
    EQUINOX.car_info,
    EQUINOX.specs,
  )
  SUBURBAN = GMPlatformConfig(
    "CHEVROLET SUBURBAN PREMIER 2016",
    GMCarInfo("Chevrolet Suburban Premier 2016-2020"),
    CarSpecs(mass=2731, wheelbase=3.302, steerRatio=17.3, centerToFrontRatio=0.49),
  )
  SUBURBAN_CC = GMPlatformConfig(
    "CHEVROLET SUBURBAN NO ACC",
    SUBURBAN.car_info,
    SUBURBAN.specs,
  )
  YUKON_CC = GMPlatformConfig(
    "GMC YUKON NO ACC",
    GMCarInfo("GMC Yukon No ACC"),
    CarSpecs(mass=2541, wheelbase=2.95, steerRatio=16.3, centerToFrontRatio=0.4),
  )
  CT6_CC = GMPlatformConfig(
    "CADILLAC CT6 NO ACC",
    GMCarInfo("Cadillac CT6 No ACC"),
    CarSpecs(mass=2358, wheelbase=3.11, steerRatio=17.7, centerToFrontRatio=0.4),
  )
  TRAILBLAZER_CC = GMPlatformConfig(
    "CHEVROLET TRAILBLAZER NO ACC",
    TRAILBLAZER.car_info,
    TRAILBLAZER.specs,
  )
  XT4 = GMPlatformConfig(
    "CADILLAC XT4 2023",
    GMCarInfo("Cadillac XT4 2023", "Driver Assist Package"),
    CarSpecs(mass=1660, wheelbase=2.78, steerRatio=14.4, centerToFrontRatio=0.4),
  )


class CruiseButtons:
  INIT = 0
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  MAIN = 5
  CANCEL = 6

class AccState:
  OFF = 0
  ACTIVE = 1
  FAULTED = 3
  STANDSTILL = 4

class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CAMERA = 2
  CHASSIS = 2
  LOOPBACK = 128
  DROPPED = 192

class GMFlags(IntFlag):
  PEDAL_LONG = 1
  CC_LONG = 2
  NO_CAMERA = 4
  NO_ACCELERATOR_POS_MSG = 8


# In a Data Module, an identifier is a string used to recognize an object,
# either by itself or together with the identifiers of parent objects.
# Each returns a 4 byte hex representation of the decimal part number. `b"\x02\x8c\xf0'"` -> 42790951
GM_BOOT_SOFTWARE_PART_NUMER_REQUEST = b'\x1a\xc0'  # likely does not contain anything useful
GM_SOFTWARE_MODULE_1_REQUEST = b'\x1a\xc1'
GM_SOFTWARE_MODULE_2_REQUEST = b'\x1a\xc2'
GM_SOFTWARE_MODULE_3_REQUEST = b'\x1a\xc3'

# Part number of XML data file that is used to configure ECU
GM_XML_DATA_FILE_PART_NUMBER = b'\x1a\x9c'
GM_XML_CONFIG_COMPAT_ID = b'\x1a\x9b'  # used to know if XML file is compatible with the ECU software/hardware

# This DID is for identifying the part number that reflects the mix of hardware,
# software, and calibrations in the ECU when it first arrives at the vehicle assembly plant.
# If there's an Alpha Code, it's associated with this part number and stored in the DID $DB.
GM_END_MODEL_PART_NUMBER_REQUEST = b'\x1a\xcb'
GM_END_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST = b'\x1a\xdb'
GM_BASE_MODEL_PART_NUMBER_REQUEST = b'\x1a\xcc'
GM_BASE_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST = b'\x1a\xdc'
GM_FW_RESPONSE = b'\x5a'

GM_FW_REQUESTS = [
  GM_BOOT_SOFTWARE_PART_NUMER_REQUEST,
  GM_SOFTWARE_MODULE_1_REQUEST,
  GM_SOFTWARE_MODULE_2_REQUEST,
  GM_SOFTWARE_MODULE_3_REQUEST,
  GM_XML_DATA_FILE_PART_NUMBER,
  GM_XML_CONFIG_COMPAT_ID,
  GM_END_MODEL_PART_NUMBER_REQUEST,
  GM_END_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST,
  GM_BASE_MODEL_PART_NUMBER_REQUEST,
  GM_BASE_MODEL_PART_NUMBER_ALPHA_CODE_REQUEST,
]

GM_RX_OFFSET = 0x400

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[request for req in GM_FW_REQUESTS for request in [
    Request(
      [StdQueries.SHORT_TESTER_PRESENT_REQUEST, req],
      [StdQueries.SHORT_TESTER_PRESENT_RESPONSE, GM_FW_RESPONSE + bytes([req[-1]])],
      rx_offset=GM_RX_OFFSET,
      bus=0,
      logging=True,
    ),
  ]],
  extra_ecus=[(Ecu.fwdCamera, 0x24b, None)],
)

EV_CAR = {CAR.VOLT, CAR.BOLT_EUV, CAR.VOLT_CC, CAR.BOLT_CC}
CC_ONLY_CAR = {CAR.VOLT_CC, CAR.BOLT_CC, CAR.EQUINOX_CC, CAR.SUBURBAN_CC, CAR.YUKON_CC, CAR.CT6_CC, CAR.TRAILBLAZER_CC}

# We're integrated at the Safety Data Gateway Module on these cars
SDGM_CAR = {CAR.XT4}

# We're integrated at the camera with VOACC on these cars (instead of ASCM w/ OBD-II harness)
CAMERA_ACC_CAR = {CAR.BOLT_EUV, CAR.SILVERADO, CAR.EQUINOX, CAR.TRAILBLAZER}
CAMERA_ACC_CAR.update({CAR.VOLT_CC, CAR.BOLT_CC, CAR.EQUINOX_CC, CAR.YUKON_CC, CAR.CT6_CC, CAR.TRAILBLAZER_CC})

STEER_THRESHOLD = 1.0

DBC = CAR.create_dbc_map()

'''
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Union

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.car.docs_definitions import CarFootnote, CarInfo, CarPart, CarParts, Column
Ecu = car.CarParams.Ecu


class CarControllerParams:
  STEER_MAX = 300  # GM limit is 3Nm. Used by carcontroller to generate LKA output
  STEER_STEP = 3  # Active control frames per command (~33hz)
  INACTIVE_STEER_STEP = 10  # Inactive control frames per command (10hz)
  STEER_DELTA_UP = 10  # Delta rates require review due to observed EPS weakness
  STEER_DELTA_DOWN = 15
  STEER_DRIVER_ALLOWANCE = 65
  STEER_DRIVER_MULTIPLIER = 4
  STEER_DRIVER_FACTOR = 100
  NEAR_STOP_BRAKE_PHASE = 0.5  # m/s

  # Heartbeat for dash "Service Adaptive Cruise" and "Service Front Camera"
  ADAS_KEEPALIVE_STEP = 100
  CAMERA_KEEPALIVE_STEP = 100

  # Allow small margin below -3.5 m/s^2 from ISO 15622:2018 since we
  # perform the closed loop control, and might need some
  # to apply some more braking if we're on a downhill slope.
  # Our controller should still keep the 2 second average above
  # -3.5 m/s^2 as per planner limits
  ACCEL_MAX = 2.  # m/s^2
  ACCEL_MIN = -4.  # m/s^2

  def __init__(self, CP):
    # Gas/brake lookups
    self.ZERO_GAS = 2048  # Coasting
    self.MAX_BRAKE = 400  # ~ -4.0 m/s^2 with regen

    if CP.carFingerprint in CAMERA_ACC_CAR:
      self.MAX_GAS = 3400
      self.MAX_ACC_REGEN = 1514
      self.INACTIVE_REGEN = 1554
      # Camera ACC vehicles have no regen while enabled.
      # Camera transitions to MAX_ACC_REGEN from ZERO_GAS and uses friction brakes instantly
      max_regen_acceleration = 0.

    else:
      self.MAX_GAS = 3072  # Safety limit, not ACC max. Stock ACC >4096 from standstill.
      self.MAX_ACC_REGEN = 1404  # Max ACC regen is slightly less than max paddle regen
      self.INACTIVE_REGEN = 1404
      # ICE has much less engine braking force compared to regen in EVs,
      # lower threshold removes some braking deadzone
      max_regen_acceleration = -1. if CP.carFingerprint in EV_CAR else -0.1

    self.GAS_LOOKUP_BP = [max_regen_acceleration, 0., self.ACCEL_MAX]
    self.GAS_LOOKUP_V = [self.MAX_ACC_REGEN, self.ZERO_GAS, self.MAX_GAS]

    self.BRAKE_LOOKUP_BP = [self.ACCEL_MIN, max_regen_acceleration]
    self.BRAKE_LOOKUP_V = [self.MAX_BRAKE, 0.]


class CAR:
  HOLDEN_ASTRA = "HOLDEN ASTRA RS-V BK 2017"
  VOLT = "CHEVROLET VOLT PREMIER 2017"
  CADILLAC_ATS = "CADILLAC ATS Premium Performance 2018"
  MALIBU = "CHEVROLET MALIBU PREMIER 2017"
  ACADIA = "GMC ACADIA DENALI 2018"
  BUICK_LACROSSE = "BUICK LACROSSE 2017"
  BUICK_REGAL = "BUICK REGAL ESSENCE 2018"
  ESCALADE = "CADILLAC ESCALADE 2017"
  ESCALADE_ESV = "CADILLAC ESCALADE ESV 2016"
  BOLT_EUV = "CHEVROLET BOLT EUV 2022"
  SILVERADO = "CHEVROLET SILVERADO 1500 2020"
  EQUINOX = "CHEVROLET EQUINOX 2019"
  TRAILBLAZER = "CHEVROLET TRAILBLAZER 2021"


class Footnote(Enum):
  OBD_II = CarFootnote(
    'Requires a <a href="https://github.com/commaai/openpilot/wiki/GM#hardware" target="_blank">community built ASCM harness</a>. ' +
    '<b><i>NOTE: disconnecting the ASCM disables Automatic Emergency Braking (AEB).</i></b>',
    Column.MODEL)


@dataclass
class GMCarInfo(CarInfo):
  package: str = "Adaptive Cruise Control (ACC)"

  def init_make(self, CP: car.CarParams):
    if CP.networkLocation == car.CarParams.NetworkLocation.fwdCamera:
      self.car_parts = CarParts.common([CarPart.gm])
    else:
      self.car_parts = CarParts([CarPart.obd_ii, CarPart.long_obdc_cable, CarPart.usbc_coupler, CarPart.mount, CarPart.right_angle_obd_c_cable_1_5ft])
      self.footnotes.append(Footnote.OBD_II)


CAR_INFO: Dict[str, Union[GMCarInfo, List[GMCarInfo]]] = {
  CAR.HOLDEN_ASTRA: GMCarInfo("Holden Astra 2017"),
  CAR.VOLT: GMCarInfo("Chevrolet Volt 2017-18", min_enable_speed=0, video_link="https://youtu.be/QeMCN_4TFfQ"),
  CAR.CADILLAC_ATS: GMCarInfo("Cadillac ATS Premium Performance 2018"),
  CAR.MALIBU: GMCarInfo("Chevrolet Malibu Premier 2017"),
  CAR.ACADIA: GMCarInfo("GMC Acadia 2018", video_link="https://www.youtube.com/watch?v=0ZN6DdsBUZo"),
  CAR.BUICK_LACROSSE: GMCarInfo("Buick LaCrosse 2017-19", "Driver Confidence Package 2"),
  CAR.BUICK_REGAL: GMCarInfo("Buick Regal Essence 2018"),
  CAR.ESCALADE: GMCarInfo("Cadillac Escalade 2017", "Driver Assist Package"),
  CAR.ESCALADE_ESV: GMCarInfo("Cadillac Escalade ESV 2016", "Adaptive Cruise Control (ACC) & LKAS"),
  CAR.BOLT_EUV: [
    GMCarInfo("Chevrolet Bolt EUV 2022-23", "Premier or Premier Redline Trim without Super Cruise Package", video_link="https://youtu.be/xvwzGMUA210"),
    GMCarInfo("Chevrolet Bolt EV 2022-23", "2LT Trim with Adaptive Cruise Control Package"),
  ],
  CAR.SILVERADO: [
    GMCarInfo("Chevrolet Silverado 1500 2020-21", "Safety Package II"),
    GMCarInfo("GMC Sierra 1500 2020-21", "Driver Alert Package II", video_link="https://youtu.be/5HbNoBLzRwE"),
  ],
  CAR.EQUINOX: GMCarInfo("Chevrolet Equinox 2019-22"),
  CAR.TRAILBLAZER: GMCarInfo("Chevrolet Trailblazer 2021-22"),
}


class CruiseButtons:
  INIT = 0
  UNPRESS = 1
  RES_ACCEL = 2
  DECEL_SET = 3
  MAIN = 5
  CANCEL = 6

class AccState:
  OFF = 0
  ACTIVE = 1
  FAULTED = 3
  STANDSTILL = 4

class CanBus:
  POWERTRAIN = 0
  OBSTACLE = 1
  CAMERA = 2
  CHASSIS = 2
  SW_GMLAN = 3
  LOOPBACK = 128
  DROPPED = 192

FINGERPRINTS = {
  CAR.HOLDEN_ASTRA: [
  # Astra BK MY17, ASCM unplugged
  {
    190: 8, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 8, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 393: 8, 398: 8, 401: 8, 413: 8, 417: 8, 419: 8, 422: 1, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 8, 455: 7, 456: 8, 458: 5, 479: 8, 481: 7, 485: 8, 489: 8, 497: 8, 499: 3, 500: 8, 501: 8, 508: 8, 528: 5, 532: 6, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 647: 5, 707: 8, 715: 8, 723: 8, 753: 5, 761: 7, 806: 1, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 961: 8, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1009: 8, 1011: 6, 1017: 8, 1019: 3, 1020: 8, 1105: 6, 1217: 8, 1221: 5, 1225: 8, 1233: 8, 1249: 8, 1257: 6, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 8, 1280: 4, 1300: 8, 1328: 4, 1417: 8, 1906: 7, 1907: 7, 1908: 7, 1912: 7, 1919: 7,
  }],
  CAR.VOLT: [
  # Volt Premier w/ ACC 2017
  {
    170: 8, 171: 8, 189: 7, 190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 288: 5, 289: 8, 298: 8, 304: 1, 308: 4, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 389: 2, 390: 7, 417: 7, 419: 1, 426: 7, 451: 8, 452: 8, 453: 6, 454: 8, 456: 8, 479: 3, 481: 7, 485: 8, 489: 8, 493: 8, 495: 4, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 528: 4, 532: 6, 546: 7, 550: 8, 554: 3, 558: 8, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 566: 5, 567: 3, 568: 1, 573: 1, 577: 8, 647: 3, 707: 8, 711: 6, 715: 8, 761: 7, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 961: 8, 969: 8, 977: 8, 979: 7, 988: 6, 989: 8, 995: 7, 1001: 8, 1005: 6, 1009: 8, 1017: 8, 1019: 2, 1020: 8, 1105: 6, 1187: 4, 1217: 8, 1221: 5, 1223: 3, 1225: 7, 1227: 4, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1273: 3, 1275: 3, 1280: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1601: 8, 1905: 7, 1906: 7, 1907: 7, 1910: 7, 1912: 7, 1922: 7, 1927: 7, 1928: 7, 2016: 8, 2020: 8, 2024: 8, 2028: 8
  },
  # Volt Premier w/ ACC 2018
  {
    170: 8, 171: 8, 189: 7, 190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 288: 5, 298: 8, 304: 1, 308: 4, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 389: 2, 390: 7, 417: 7, 419: 1, 426: 7, 451: 8, 452: 8, 453: 6, 454: 8, 456: 8, 479: 3, 481: 7, 485: 8, 489: 8, 493: 8, 495: 4, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 528: 4, 532: 6, 546: 7, 550: 8, 554: 3, 558: 8, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 566: 5, 567: 3, 568: 1, 573: 1, 577: 8, 578: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 3, 707: 8, 711: 6, 715: 8, 717: 5, 761: 7, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 961: 8, 967: 4, 969: 8, 977: 8, 979: 7, 988: 6, 989: 8, 995: 7, 1001: 8, 1005: 6, 1009: 8, 1017: 8, 1019: 2, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1187: 4, 1217: 8, 1221: 5, 1223: 3, 1225: 7, 1227: 4, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1273: 3, 1275: 3, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1516: 8, 1601: 8, 1618: 8, 1905: 7, 1906: 7, 1907: 7, 1910: 7, 1912: 7, 1922: 7, 1927: 7, 1930: 7, 2016: 8, 2018: 8, 2020: 8, 2024: 8, 2028: 8
  },
  # Volt Premier 2018 w/ flashed firmware, no radar
  {
    170: 8, 171: 8, 189: 7, 190: 6, 192: 5, 193: 8, 197: 8, 199: 4, 201: 6, 209: 7, 211: 2, 241: 6, 288: 5, 289: 1, 290: 1, 298: 2, 304: 1, 308: 4, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 368: 8, 381: 2, 384: 8, 386: 5, 388: 8, 389: 2, 390: 7, 417: 7, 419: 1, 426: 7, 451: 8, 452: 8, 453: 6, 454: 8, 456: 8, 458: 8, 479: 3, 481: 7, 485: 8, 489: 5, 493: 8, 495: 4, 497: 8, 499: 3, 500: 6, 501: 3, 508: 8, 512: 3, 528: 4, 530: 8, 532: 6, 537: 5, 539: 8, 542: 7, 546: 7, 550: 8, 554: 3, 558: 8, 560: 6, 562: 4, 563: 5, 564: 5, 565: 5, 566: 5, 567: 3, 568: 1, 573: 1, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 3, 707: 8, 711: 6, 761: 7, 810: 8, 821: 4, 823: 7, 832: 8, 840: 5, 842: 5, 844: 8, 853: 8, 866: 4, 961: 8, 967: 4, 969: 8, 977: 8, 979: 7, 988: 6, 989: 8, 995: 7, 1001: 5, 1003: 5, 1005: 6, 1009: 8, 1017: 8, 1019: 2, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1187: 4, 1217: 8, 1221: 5, 1223: 3, 1225: 7, 1227: 4, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1273: 3, 1275: 3, 1280: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1905: 7, 1906: 7, 1907: 7, 1910: 7, 1912: 7, 1922: 7, 1927: 7
  }],
  CAR.BUICK_LACROSSE: [
  # LaCrosse Premium AWD 2017
  {
    190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 322: 7, 328: 1, 352: 5, 353: 3, 381: 6, 386: 8, 388: 8, 393: 7, 398: 8, 407: 7, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 455: 7, 456: 8, 463: 3, 479: 3, 481: 7, 485: 8, 487: 8, 489: 8, 495: 4, 497: 8, 499: 3, 500: 6, 501: 8, 503: 1, 508: 8, 510: 8, 528: 5, 532: 6, 534: 2, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 573: 1, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 5, 707: 8, 753: 5, 761: 7, 801: 8, 804: 3, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 872: 1, 882: 8, 890: 1, 892: 2, 893: 1, 894: 1, 961: 8, 967: 4, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1005: 6, 1009: 8, 1011: 6, 1013: 3, 1017: 8, 1019: 2, 1020: 8, 1022: 1, 1105: 6, 1217: 8, 1221: 5, 1223: 2, 1225: 7, 1233: 8, 1243: 3, 1249: 8, 1257: 6, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1280: 4, 1300: 8, 1322: 6, 1328: 4, 1417: 8, 1609: 8, 1613: 8, 1649: 8, 1792: 8, 1798: 8, 1824: 8, 1825: 8, 1840: 8, 1842: 8, 1858: 8, 1860: 8, 1863: 8, 1872: 8, 1875: 8, 1882: 8, 1888: 8, 1889: 8, 1892: 8, 1904: 7, 1906: 7, 1907: 7, 1912: 7, 1913: 7, 1914: 7, 1916: 7, 1918: 7, 1919: 7, 1937: 8, 1953: 8, 1968: 8, 2001: 8, 2017: 8, 2018: 8, 2020: 8, 2026: 8
  }],
  CAR.BUICK_REGAL : [
  # Regal TourX Essence w/ ACC 2018
  {
    190: 8, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 8, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 322: 7, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 393: 7, 398: 8, 407: 7, 413: 8, 417: 8, 419: 8, 422: 4, 426: 8, 431: 8, 442: 8, 451: 8, 452: 8, 453: 8, 455: 7, 456: 8, 463: 3, 479: 8, 481: 7, 485: 8, 487: 8, 489: 8, 495: 8, 497: 8, 499: 3, 500: 8, 501: 8, 508: 8, 528: 5, 532: 6, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 569: 3, 573: 1, 577: 8, 578: 8, 579: 8, 587: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 3, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 882: 8, 884: 8, 890: 1, 892: 2, 893: 2, 894: 1, 961: 8, 967: 8, 969: 8, 977: 8, 979: 8, 985: 8, 1001: 8, 1005: 6, 1009: 8, 1011: 8, 1013: 3, 1017: 8, 1020: 8, 1024: 8, 1025: 8, 1026: 8, 1027: 8, 1028: 8, 1029: 8, 1030: 8, 1031: 8, 1032: 2, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1223: 8, 1225: 7, 1233: 8, 1249: 8, 1257: 6, 1259: 8, 1261: 8, 1263: 8, 1265: 8, 1267: 8, 1271: 8, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1328: 4, 1417: 8, 1601: 8, 1602: 8, 1603: 7, 1611: 8, 1618: 8, 1906: 8, 1907: 7, 1912: 7, 1914: 7, 1916: 7, 1919: 7, 1930: 7, 2016: 8, 2018: 8, 2019: 8, 2024: 8, 2026: 8
  }],
  CAR.CADILLAC_ATS: [
  # Cadillac ATS Coupe Premium Performance 3.6L RWD w/ ACC 2018
  {
    190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 322: 7, 328: 1, 352: 5, 368: 3, 381: 6, 384: 4, 386: 8, 388: 8, 393: 7, 398: 8, 401: 8, 407: 7, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 455: 7, 456: 8, 462: 4, 479: 3, 481: 7, 485: 8, 487: 8, 489: 8, 491: 2, 493: 8, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 528: 5, 532: 6, 534: 2, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 573: 1, 577: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 6, 707: 8, 715: 8, 717: 5, 719: 5, 723: 2, 753: 5, 761: 7, 801: 8, 804: 3, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 882: 8, 890: 1, 892: 2, 893: 2, 894: 1, 961: 8, 967: 4, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1005: 6, 1009: 8, 1011: 6, 1013: 3, 1017: 8, 1019: 2, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1223: 3, 1225: 7, 1233: 8, 1241: 3, 1249: 8, 1257: 6, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1271: 8, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1601: 8, 1904: 7, 1906: 7, 1907: 7, 1912: 7, 1916: 7, 1917: 7, 1918: 7, 1919: 7, 1920: 7, 1930: 7, 2016: 8, 2024: 8
  }],
  CAR.MALIBU: [
  # Malibu Premier w/ ACC 2017
  {
    190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 393: 7, 398: 8, 407: 7, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 455: 7, 456: 8, 479: 3, 481: 7, 485: 8, 487: 8, 489: 8, 495: 4, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 528: 5, 532: 6, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 565: 5, 567: 5, 573: 1, 577: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 6, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 961: 8, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1005: 6, 1009: 8, 1013: 3, 1017: 8, 1019: 2, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1223: 2, 1225: 7, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1601: 8, 1906: 7, 1907: 7, 1912: 7, 1919: 7, 1930: 7, 2016: 8, 2024: 8,
  }],
  CAR.ACADIA: [
  # Acadia Denali w/ACC 2018
  {
    190: 6, 192: 5, 193: 8, 197: 8, 199: 4, 201: 6, 208: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 289: 1, 290: 1, 298: 8, 304: 8, 309: 8, 313: 8, 320: 8, 322: 7, 328: 1, 352: 7, 368: 8, 381: 8, 384: 8, 386: 8, 388: 8, 393: 8, 398: 8, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 454: 8, 455: 7, 458: 8, 460: 4, 462: 4, 463: 3, 479: 3, 481: 7, 485: 8, 489: 5, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 512: 3, 530: 8, 532: 6, 534: 2, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 567: 5, 568: 2, 573: 1, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 6, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 789: 5, 800: 6, 801: 8, 803: 8, 804: 3, 805: 8, 832: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 961: 8, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1003: 5, 1005: 6, 1009: 8, 1017: 8, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1225: 8, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1328: 4, 1417: 8, 1906: 7, 1907: 7, 1912: 7, 1914: 7, 1918: 7, 1919: 7, 1920: 7, 1930: 7
  },
  # Acadia Denali w/ /ACC 2018
  {
    190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 208: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 289: 8, 298: 8, 304: 1, 309: 8, 313: 8, 320: 3, 322: 7, 328: 1, 338: 6, 340: 6, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 393: 8, 398: 8, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 454: 8, 455: 7, 462: 4, 463: 3, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 532: 6, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 567: 5, 573: 1, 577: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 6, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 961: 8, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1005: 6, 1009: 8, 1017: 8, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1225: 8, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1328: 4, 1417: 8, 1601: 8, 1906: 7, 1907: 7, 1912: 7, 1914: 7, 1919: 7, 1920: 7, 1930: 7, 2016: 8, 2024: 8
  }],
  CAR.ESCALADE: [
  {
    170: 8, 190: 6, 193: 8, 197: 8, 199: 4, 201: 8, 208: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 322: 7, 328: 1, 352: 5, 381: 6, 384: 4, 386: 8, 388: 8, 393: 7, 398: 8, 407: 4, 413: 8, 417: 7, 419: 1, 422: 4, 426: 7, 431: 8, 442: 8, 451: 8, 452: 8, 453: 6, 454: 8, 455: 7, 460: 5, 462: 4, 463: 3, 479: 3, 481: 7, 485: 8, 487: 8, 489: 8, 497: 8, 499: 3, 500: 6, 501: 8, 508: 8, 510: 8, 532: 6, 534: 2, 554: 3, 560: 8, 562: 8, 563: 5, 564: 5, 573: 1, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 647: 6, 707: 8, 715: 8, 717: 5, 719: 5, 761: 7, 801: 8, 804: 3, 810: 8, 840: 5, 842: 5, 844: 8, 866: 4, 869: 4, 880: 6, 961: 8, 967: 4, 969: 8, 977: 8, 979: 8, 985: 5, 1001: 8, 1005: 6, 1009: 8, 1017: 8, 1019: 2, 1020: 8, 1033: 7, 1034: 7, 1105: 6, 1217: 8, 1221: 5, 1223: 2, 1225: 7, 1233: 8, 1249: 8, 1257: 6, 1265: 8, 1267: 1, 1280: 4, 1296: 4, 1300: 8, 1322: 6, 1323: 4, 1328: 4, 1417: 8, 1609: 8, 1613: 8, 1649: 8, 1792: 8, 1798: 8, 1824: 8, 1825: 8, 1840: 8, 1842: 8, 1858: 8, 1860: 8, 1863: 8, 1872: 8, 1875: 8, 1882: 8, 1888: 8, 1889: 8, 1892: 8, 1906: 7, 1907: 7, 1912: 7, 1914: 7, 1917: 7, 1918: 7, 1919: 7, 1920: 7, 1930: 7, 1937: 8, 1953: 8, 1968: 8, 2001: 8, 2017: 8, 2018: 8, 2020: 8, 2026: 8
  }],
  CAR.ESCALADE_ESV: [
  {
    309: 1, 848: 8, 849: 8, 850: 8, 851: 8, 852: 8, 853: 8, 854: 3, 1056: 6, 1057: 8, 1058: 8, 1059: 8, 1060: 8, 1061: 8, 1062: 8, 1063: 8, 1064: 8, 1065: 8, 1066: 8, 1067: 8, 1068: 8, 1120: 8, 1121: 8, 1122: 8, 1123: 8, 1124: 8, 1125: 8, 1126: 8, 1127: 8, 1128: 8, 1129: 8, 1130: 8, 1131: 8, 1132: 8, 1133: 8, 1134: 8, 1135: 8, 1136: 8, 1137: 8, 1138: 8, 1139: 8, 1140: 8, 1141: 8, 1142: 8, 1143: 8, 1146: 8, 1147: 8, 1148: 8, 1149: 8, 1150: 8, 1151: 8, 1216: 8, 1217: 8, 1218: 8, 1219: 8, 1220: 8, 1221: 8, 1222: 8, 1223: 8, 1224: 8, 1225: 8, 1226: 8, 1232: 8, 1233: 8, 1234: 8, 1235: 8, 1236: 8, 1237: 8, 1238: 8, 1239: 8, 1240: 8, 1241: 8, 1242: 8, 1787: 8, 1788: 8
  }],
  CAR.BOLT_EUV: [
  {
    189: 7, 190: 7, 193: 8, 197: 8, 201: 8, 209: 7, 211: 3, 241: 6, 257: 8, 288: 5, 289: 8, 298: 8, 304: 3, 309: 8, 311: 8, 313: 8, 320: 4, 322: 7, 328: 1, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 451: 8, 452: 8, 453: 6, 458: 5, 463: 3, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 500: 6, 501: 8, 528: 5, 532: 6, 560: 8, 562: 8, 563: 5, 565: 5, 566: 8, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 789: 5, 800: 6, 810: 8, 840: 5, 842: 5, 844: 8, 848: 4, 869: 4, 880: 6, 977: 8, 1001: 8, 1017: 8, 1020: 8, 1217: 8, 1221: 5, 1233: 8, 1249: 8, 1265: 8, 1280: 4, 1296: 4, 1300: 8, 1930: 7
  }],
  CAR.SILVERADO: [
  {
    190: 6, 193: 8, 197: 8, 201: 8, 208: 8, 209: 7, 211: 2, 241: 6, 249: 8, 257: 8, 288: 5, 289: 8, 298: 8, 304: 3, 309: 8, 311: 8, 313: 8, 320: 4, 322: 7, 328: 1, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 413: 8, 451: 8, 452: 8, 453: 6, 455: 7, 460: 5, 463: 3, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 500: 6, 501: 8, 528: 5, 532: 6, 534: 2, 560: 8, 562: 8, 563: 5, 565: 5, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 707: 8, 715: 8, 717: 5, 761: 7, 789: 5, 800: 6, 801: 8, 810: 8, 840: 5, 842: 5, 844: 8, 848: 4, 869: 4, 880: 6, 977: 8, 1001: 8, 1011: 6, 1017: 8, 1020: 8, 1033: 7, 1034: 7, 1217: 8, 1221: 5, 1233: 8, 1249: 8, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1271: 8, 1280: 4, 1296: 4, 1300: 8, 1930: 7
  }],
  CAR.EQUINOX: [
  {
    190: 6, 193: 8, 197: 8, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 257: 8, 288: 5, 289: 8, 298: 8, 304: 1, 309: 8, 311: 8, 313: 8, 320: 3, 328: 1, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 413: 8, 451: 8, 452: 8, 453: 6, 455: 7, 463: 3, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 500: 6, 501: 8, 510: 8, 528: 5, 532: 6, 560: 8, 562: 8, 563: 5, 565: 5, 608: 8, 609: 6, 610: 6, 611: 6, 612: 8, 613: 8, 707: 8, 715: 8, 717: 5, 753: 5, 761: 7, 789: 5, 800: 6, 810: 8, 840: 5, 842: 5, 844: 8, 869: 4, 880: 6, 977: 8, 1001: 8, 1011: 6, 1017: 8, 1020: 8, 1033: 7, 1034: 7, 1217: 8, 1221: 5, 1233: 8, 1249: 8, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1271: 8, 1280: 4, 1296: 4, 1300: 8, 1930: 7
  }],
  # Trailblazer also matches as a Silverado, so comment out to avoid conflicts.
  # TODO: split with FW versions
  CAR.TRAILBLAZER: [
  {
  #   190: 6, 193: 8, 197: 8, 201: 8, 209: 7, 211: 2, 241: 6, 249: 8, 288: 5, 289: 8, 298: 8, 304: 3, 309: 8, 311: 8, 313: 8, 320: 4, 328: 1, 352: 5, 381: 8, 384: 4, 386: 8, 388: 8, 413: 8, 451: 8, 452: 8, 453: 6, 455: 7, 479: 3, 481: 7, 485: 8, 489: 8, 497: 8, 500: 6, 501: 8, 532: 6, 560: 8, 562: 8, 563: 5, 565: 5, 707: 8, 715: 8, 717: 5, 761: 7, 789: 5, 800: 6, 810: 8, 840: 5, 842: 5, 844: 8, 869: 4, 880: 6, 977: 8, 1001: 8, 1011: 6, 1017: 8, 1020: 8, 1217: 8, 1221: 5, 1233: 8, 1249: 8, 1259: 8, 1261: 7, 1263: 4, 1265: 8, 1267: 1, 1271: 8, 1280: 4, 1296: 4, 1300: 8, 1609: 8, 1613: 8, 1649: 8, 1792: 8, 1798: 8, 1824: 8, 1825: 8, 1840: 8, 1842: 8, 1858: 8, 1860: 8, 1863: 8, 1872: 8, 1875: 8, 1882: 8, 1888: 8, 1889: 8, 1892: 8, 1930: 7, 1937: 8, 1953: 8, 1968: 8, 2001: 8, 2017: 8, 2018: 8, 2020: 8
  }],
}

DBC: Dict[str, Dict[str, str]] = defaultdict(lambda: dbc_dict('gm_global_a_powertrain_generated', 'gm_global_a_object', chassis_dbc='gm_global_a_chassis'))

EV_CAR = {CAR.VOLT, CAR.BOLT_EUV}

# We're integrated at the camera with VOACC on these cars (instead of ASCM w/ OBD-II harness)
CAMERA_ACC_CAR = {CAR.BOLT_EUV, CAR.SILVERADO, CAR.EQUINOX, CAR.TRAILBLAZER}

STEER_THRESHOLD = 1.0
'''
