# hard-forked from https://github.com/commaai/openpilot/tree/05b37552f3a38f914af41f44ccc7c633ad152a15/selfdrive/controls/lib/lane_planner.py
import numpy as np
import statistics
from cereal import log
from common.filter_simple import FirstOrderFilter
from common.numpy_fast import interp
from common.realtime import DT_MDL
from system.swaglog import cloudlog
from common.logger import sLogger

# positive numbers go right, negative go left
TRAJECTORY_SIZE = 33
PATH_OFFSET = 0.2
CAMERA_OFFSET = 0.2

def lerp(a, b, t):
  return (b * t) + (a * (1.0 - t))

def clamp(num, min_value, max_value):
  return max(min(num, max_value), min_value)

class LanePlanner:
  def __init__(self, wide_camera=False):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.lane_width_estimate = FirstOrderFilter(3.1, 9.95, DT_MDL)
    self.lane_width_certainty = FirstOrderFilter(1.0, 0.95, DT_MDL)
    self.lane_width = 3.7

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lle_y = np.zeros((TRAJECTORY_SIZE,))
    self.rle_y = np.zeros((TRAJECTORY_SIZE,))
    self.lle_y_dists = []
    self.rle_y_dists = []

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.camera_offset = -CAMERA_OFFSET if wide_camera else CAMERA_OFFSET
    self.path_offset = -PATH_OFFSET if wide_camera else PATH_OFFSET

  def parse_model(self, md):
    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.lle_y = np.array(edges[0].y) + self.camera_offset
      self.rle_y = np.array(edges[1].y) + self.camera_offset
      # track how far on average we are from the road edge
      # store the last few readings for averaging
      self.lle_y_dists.append(self.lle_y[0])
      self.rle_y_dists.append(self.rle_y[0])
      if len(self.lle_y_dists) > 10:
        self.lle_y_dists.pop(0)
        self.rle_y_dists.pop(0)

    if len(lane_lines) == 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y) + self.camera_offset
      self.rll_y = np.array(lane_lines[2].y) + self.camera_offset
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.LateralPlan.Desire.laneChangeRight]

  def get_d_path(self, v_ego, path_t, path_xyz):
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    path_xyz[:, 1] += self.path_offset
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      prob_mods.append(interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    # Find current lanewidth
    self.lane_width_certainty.update(l_prob * r_prob)
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])
    self.lane_width_estimate.update(current_lane_width)
    speed_lane_width = interp(v_ego, [0., 31.], [2.8, 3.5])
    self.lane_width = lerp(speed_lane_width, self.lane_width_estimate.x, self.lane_width_certainty.x)

    clipped_lane_width = min(4.0, self.lane_width)

    # lane paths
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # which edge are we closest to? pick a path from it
    left_edge_dist = clamp(statistics.fmean(self.lle_y_dists), -4.0, -1.2)
    right_edge_dist = clamp(statistics.fmean(self.rle_y_dists), 1.2,  4.0)
    path_from_edge = self.lle_y - left_edge_dist if abs(left_edge_dist) < abs(right_edge_dist) else self.rle_y - right_edge_dist

    # ok, which path will we use?
    lane_path_prob = max(l_prob, r_prob)
    lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)
    final_path_y = lerp(path_from_edge, lane_path_y, lane_path_prob)

    #debug
    sLogger.Send("0lP" + "{:.2f}".format(l_prob) + " rP" + "{:.2f}".format(r_prob) +
                " lX" + "{:.1f}".format(self.lll_y[0]) + " rX" + "{:.1f}".format(self.rll_y[0]) +
                " leX" + "{:.1f}".format(self.lle_y[0]) + " reX" + "{:.1f}".format(self.rle_y[0]) +
                " ls" + "{:.2f}".format(self.lll_std) + " rs" + "{:.2f}".format(self.rll_std) +
                " w" + "{:.1f}".format(self.lane_width) + " ld" + "{:.1f}".format(left_edge_dist) +
                " rd" + "{:.1f}".format(right_edge_dist))

    # only switch to laneless if we can't see any edges or lanes
    if left_edge_dist > -4.0 or right_edge_dist < 4.0 or lane_path_prob > 0.3:
      self.d_prob = 1.0
      safe_idxs = np.isfinite(self.ll_t)
      if safe_idxs[0]:
        path_xyz[:,1] = np.interp(path_t, self.ll_t[safe_idxs], final_path_y[safe_idxs])

    return path_xyz
