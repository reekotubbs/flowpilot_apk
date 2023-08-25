# hard-forked from https://github.com/commaai/openpilot/blob/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/locationd/calibrationd.py
import cereal.messaging as messaging
from cereal import log
from common.realtime import set_realtime_priority
from common.params import Params
from common.conversions import Conversions as CV
from common.transformations.camera import get_view_frame_from_road_frame
from common.transformations.orientation import rot_from_euler, euler_from_rot
import numpy as np
import os
from system.swaglog import cloudlog


MIN_SPEED_FILTER = 15 * CV.MPH_TO_MS
MAX_VEL_ANGLE_STD = np.radians(0.25)
MAX_YAW_RATE_FILTER = np.radians(2.9)  # per second # TODO 2 rejects all readings

# This is at model frequency, blocks needed for efficiency
SMOOTH_CYCLES = 400
BLOCK_SIZE = 100
INPUTS_NEEDED = 5  # Minimum blocks needed for valid calibration
INPUTS_WANTED = 50  # We want a little bit more than we need for stability
MAX_ALLOWED_SPREAD = np.radians(2)
RPY_INIT = np.array([0.0, 0.0, 0.0])

# These values are needed to accommodate biggest modelframe
PITCH_LIMITS = np.array([-0.09074112085129739, 0.14907572052989657])
YAW_LIMITS = np.array([-0.06912048084718224, 0.06912048084718235])
DEBUG = os.getenv("DEBUG") is not None

model_height = 1.28 # kona is about this height

class Calibrator:

    def __init__(self):
        self.rpy_init = RPY_INIT.copy()
        self.valid_blocks = 0
        self.idx = 0
        self.block_idx = 0
        self.v_ego = 0.0
        self.params = Params()

        self.rpy = None
        self.rpys = None
        self.old_rpy = None

        if not self.load_cache():
            cloudlog.info("Error reading cached CalibrationParams")

        self.reset(self.rpy_init, self.valid_blocks)
        self.update_status()

    def load_cache(self):
        live_calib_bytes = self.params.get("CalibrationParams")
        if live_calib_bytes is None:
            return False
        msg = log.Event.from_bytes(live_calib_bytes)
        self.rpy_init = np.array(msg.liveCalibration.rpyCalib)
        self.valid_blocks = msg.liveCalibration.validBlocks
        return True

    def get_smooth_rpy(self):
        if self.old_rpy_weight > 0:
            return self.old_rpy_weight * self.old_rpy + (1.0 - self.old_rpy_weight) * self.rpy
        else:
            return self.rpy

    def reset(self, rpy_init=RPY_INIT, valid_blocks=0, smooth_from=None):
        if not np.isfinite(rpy_init).all():
            self.rpy = RPY_INIT.copy()
        else:
            self.rpy = rpy_init

        if not np.isfinite(valid_blocks) or valid_blocks < 0:
            self.valid_blocks = 0
        else:
            self.valid_blocks = valid_blocks
        self.rpys = np.tile(self.rpy, (INPUTS_WANTED, 1))

        self.idx = 0
        self.block_idx = 0
        self.v_ego = 0.0

        if smooth_from is None:
            self.old_rpy = RPY_INIT
            self.old_rpy_weight = 0.0
        else:
            self.old_rpy = smooth_from
            self.old_rpy_weight = 1.0

    def get_valid_idxs(self):
        # exclude current block_idx from validity window
        before_current = list(range(self.block_idx))
        after_current = list(range(min(self.valid_blocks, self.block_idx + 1), self.valid_blocks))
        return before_current + after_current

    def update_status(self):
        valid_idxs = self.get_valid_idxs()
        if valid_idxs:
            rpys = self.rpys[valid_idxs]
            self.rpy = np.mean(rpys, axis=0)
            max_rpy_calib = np.array(np.max(rpys, axis=0))
            min_rpy_calib = np.array(np.min(rpys, axis=0))
            self.calib_spread = np.abs(max_rpy_calib - min_rpy_calib)
        else:
            self.calib_spread = np.zeros(3)

        if self.valid_blocks < INPUTS_NEEDED:
            self.cal_status = log.LiveCalibrationData.Status.uncalibrated
        elif self.rpy_valid(self.rpy):
            self.cal_status = log.LiveCalibrationData.Status.calibrated
        else:
            self.cal_status = log.LiveCalibrationData.Status.invalid

        # If spread is too high, assume mounting was changed and reset to last block.
        # Make the transition smooth. Abrupt transitions are not good for feedback loop through supercombo model.
        if max(self.calib_spread) > MAX_ALLOWED_SPREAD and self.cal_status == log.LiveCalibrationData.Status.calibrated:
            self.reset(self.rpys[self.block_idx - 1], valid_blocks=INPUTS_NEEDED, smooth_from=self.rpy)

        write_this_cycle = (self.idx == 0) and (self.block_idx % (INPUTS_WANTED // 5) == 5)
        if write_this_cycle:
            self.params.put("CalibrationParams", self.get_msg().to_bytes())

    def handle_cam_odom(self, trans, rot, trans_std):
        self.old_rpy_weight = min(0.0, self.old_rpy_weight - 1 / SMOOTH_CYCLES)

        straight_and_fast = ((self.v_ego > MIN_SPEED_FILTER) and (trans[0] > MIN_SPEED_FILTER) and (
                abs(rot[2]) < MAX_YAW_RATE_FILTER))
        angle_std_threshold = MAX_VEL_ANGLE_STD
        certain_if_calib = ((np.arctan2(trans_std[1], trans[0]) < angle_std_threshold) or
                            (self.valid_blocks < INPUTS_NEEDED))

        if not straight_and_fast:
            #print("Not straight/fast! rot[2](yaw):" + "{:.2f}".format(rot[2]) + ", trans[0](speed):" + "{:.2f}".format(trans[0]))
            return None

        if not certain_if_calib:
            #print("Not certain_if_calib!")
            return None

        observed_rpy = np.array([0,
                                 -np.arctan2(trans[2], trans[0]),
                                 np.arctan2(trans[1], trans[0])])
        new_rpy = euler_from_rot(rot_from_euler(self.get_smooth_rpy()).dot(rot_from_euler(observed_rpy)))
        new_rpy = self.sanity_clip(new_rpy)

        self.rpys[self.block_idx] = (self.idx * self.rpys[self.block_idx] + (BLOCK_SIZE - self.idx) * new_rpy) / float(
            BLOCK_SIZE)
        self.idx = (self.idx + 1) % BLOCK_SIZE
        if self.idx == 0:
            self.block_idx += 1
            self.valid_blocks = max(self.block_idx, self.valid_blocks)
            self.block_idx = self.block_idx % INPUTS_WANTED

        self.update_status()
        return new_rpy

    @staticmethod
    def rpy_valid(rpy):
        if np.isnan(rpy).any():
            return False
        return (PITCH_LIMITS[0] < rpy[1] < PITCH_LIMITS[1]) and \
               (YAW_LIMITS[0] < rpy[2] < YAW_LIMITS[1])

    @staticmethod
    def sanity_clip(rpy):
        return np.array([rpy[0],
                         np.clip(rpy[1], *PITCH_LIMITS),
                         np.clip(rpy[2], *YAW_LIMITS)])

    def handle_v_ego(self, v_ego):
        self.v_ego = v_ego

    def get_extrinsic_matrix(self): # move to common
        R = rot_from_euler(self.get_smooth_rpy())
        t = np.array([[0.0, model_height, 0.0]])
        return np.vstack([R, t]).T

    def get_msg(self):
        smooth_rpy = self.get_smooth_rpy()
        extrinsic_matrix = get_view_frame_from_road_frame(0, smooth_rpy[1], smooth_rpy[2], model_height)

        msg = messaging.new_message('liveCalibration')
        liveCalibration = msg.liveCalibration

        liveCalibration.validBlocks = self.valid_blocks
        liveCalibration.calStatus = self.cal_status
        liveCalibration.calPerc = min(100 * (self.valid_blocks * BLOCK_SIZE + self.idx) // (INPUTS_NEEDED * BLOCK_SIZE), 100)
        liveCalibration.extrinsicMatrix = extrinsic_matrix.flatten().tolist()
        liveCalibration.rpyCalib = smooth_rpy.tolist()
        liveCalibration.rpyCalibSpread = self.calib_spread.tolist()

        return msg

    def send_data(self, pm):
        pm.send('liveCalibration', self.get_msg())

def calibrationd_thread(sm=None, pm=None):
    set_realtime_priority(1)

    if sm is None:
        sm = messaging.SubMaster(['cameraOdometry', 'carState'], poll=['cameraOdometry'])

    if pm is None:
        pm = messaging.PubMaster(['liveCalibration'])

    calibrator = Calibrator()

    while True:
        sm.update()

        if sm.updated['cameraOdometry'] and sm.valid['cameraOdometry']:
            calibrator.handle_v_ego(sm['carState'].vEgo)
            calibrator.handle_cam_odom(sm['cameraOdometry'].trans, sm['cameraOdometry'].rot, sm['cameraOdometry'].transStd)

        # 4Hz driven by cameraOdometry
        if sm.frame % 5 == 0:
            if calibrator.params.get_bool("ResetExtrinsicCalibration") is True:
                calibrator.reset()
                calibrator.update_status()
                calibrator.params.put_bool("ResetExtrinsicCalibration", False)
            calibrator.send_data(pm)


def main():
    calibrationd_thread()

if __name__ == "__main__":
    main()
