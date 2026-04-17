"""
Behavior state machine for autonomous mode.
Priority:
    AVOID_OBSTACLE  >  FOLLOW_LINE  >  SEARCH_LINE  >  STOPPED
"""

import time
import config


class StateMachine:
    STATES = ("FOLLOW_LINE", "AVOID_OBSTACLE", "SEARCH_LINE", "STOPPED")

    def __init__(self, robot):
        self.robot          = robot
        self.state          = "STOPPED"
        self.prev_state     = "STOPPED"
        self.last_line_pos  = "center"
        self.lost_since     = 0.0
        self.avoid_until    = 0.0
        self.obstacle_active = False

    # ------------------------------------------------------------------
    def set_state(self, new_state):
        if new_state != self.state:
            if config.VERBOSE:
                print(f"[state] {self.state} -> {new_state}")
            self.prev_state = self.state
            self.state = new_state

    # ------------------------------------------------------------------
    def update(self,
               line_position,
               line_conf,
               cam_distance_cm,
               ultrasonic_distances):
        """
        Main per-tick update.
        ultrasonic_distances : dict {left, center, right} in cm (-1 if invalid)
        """
        us = ultrasonic_distances or {}
        d_l = us.get("left",   -1.0)
        d_c = us.get("center", -1.0)
        d_r = us.get("right",  -1.0)

        # ---------- obstacle detection with hysteresis
        near_us  = (d_c > 0 and d_c < config.OBSTACLE_STOP_CM)
        near_cam = (cam_distance_cm is not None
                    and 0 < cam_distance_cm < config.CAMERA_OBSTACLE_CM)
        clear_us = (d_c < 0 or d_c > config.OBSTACLE_CLEAR_CM)

        if near_us or near_cam:
            self.obstacle_active = True
        elif clear_us:
            self.obstacle_active = False

        # ---------- decide next state
        if self.obstacle_active:
            self.set_state("AVOID_OBSTACLE")
            return self._avoid_obstacle(d_l, d_c, d_r)

        if line_position in ("left", "center", "right"):
            self.set_state("FOLLOW_LINE")
            self.last_line_pos = line_position
            self.lost_since    = 0.0
            return self._follow_line(line_position)

        # line lost
        if self.lost_since == 0.0:
            self.lost_since = time.time()
        self.set_state("SEARCH_LINE")
        return self._search_line()

    # ------------------------------------------------------------------
    def stop_autonomous(self):
        self.set_state("STOPPED")
        self.robot.stop()

    # ------------------------------------------------------------------
    # behaviors
    # ------------------------------------------------------------------
    def _follow_line(self, pos):
        if pos == "center":
            self.robot.forward()
        elif pos == "left":
            self.robot.left()
        elif pos == "right":
            self.robot.right()
        return self.state

    def _search_line(self):
        elapsed = time.time() - self.lost_since
        if elapsed < 1.5:
            # rotate toward last known direction
            if self.last_line_pos == "left":
                self.robot.left()
            elif self.last_line_pos == "right":
                self.robot.right()
            else:
                self.robot.left()
        elif elapsed < 3.0:
            self.robot.backward()
        else:
            # give up and stop briefly, then retry
            self.robot.stop()
            self.lost_since = time.time() - 1.0
        return self.state

    def _avoid_obstacle(self, d_l, d_c, d_r):
        # stop first
        self.robot.stop()

        # choose safest side
        left_ok  = d_l < 0 or d_l > config.OBSTACLE_CLEAR_CM
        right_ok = d_r < 0 or d_r > config.OBSTACLE_CLEAR_CM

        if left_ok and (not right_ok or d_l >= d_r):
            self.robot.left()
        elif right_ok:
            self.robot.right()
        else:
            # both sides blocked - back up then rotate
            self.robot.backward()
        return self.state
