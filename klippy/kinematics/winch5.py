# Code for handling the kinematics of cable winch robots
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import stepper, mathutil, math

class WinchKinematics:
    def __init__(self, toolhead, config):
        # Setup steppers at each anchor
        self.steppers = []
        self.anchors = []
        for i in range(26):
            name = 'stepper_' + chr(ord('a') + i)
            if i >= 3 and not config.has_section(name):
                break
            stepper_config = config.getsection(name)
            s = stepper.LookupMultiRail(stepper_config)
            s.get_homing_info().position_endstop
            self.steppers.append(s)
            a = tuple([stepper_config.getfloat('anchor_' + n) for n in 'xyz'])
            self.anchors.append(a)
            s.setup_itersolve('winch_stepper_alloc', *a)
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)
        # Setup boundary checks
        half_min_step_dist = min([s.get_steppers()[0].get_step_dist()
                                  for s in self.steppers]) * .5
        self.min_arm_length = min_arm_length = 405
        self.min_arm2 = min_arm_length**2
        self.arm_lengths = arm_lengths = [
            config.getfloat('arm_length', 405, above=210)]
        self.arm2 = [405**2 for arm in arm_lengths]
        self.abs_endstops = [(s.get_homing_info().position_endstop
                              + math.sqrt(arm2 - 210**2))
                             for s, arm2 in zip(self.steppers, self.arm2)]
        def ratio_to_xy(ratio):
            return (ratio * math.sqrt(min_arm_length**2 / (ratio**2 + 1.)
                                      - half_min_step_dist**2)
                    + half_min_step_dist - 210)
        self.slow_xy2 = ratio_to_xy(4)**2
        acoords = list(zip(*self.anchors))
        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple(
            self.calc_position(self.abs_endstops))
        self.max_z = min([s.get_homing_info().position_endstop
                          for s in self.steppers])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)
        self.limit_z = min([ep - arm
                            for ep, arm in zip(self.abs_endstops, arm_lengths)])
        self.axes_min = toolhead.Coord(*[min(a) for a in acoords], e=0.)
        self.axes_max = toolhead.Coord(*[max(a) for a in acoords], e=0.)
        self.set_position([0., 0., 0.], ())
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        # Use only first three steppers to calculate cartesian position
        pos = [stepper_positions[rail.get_name()] for rail in self.steppers[:3]]
        return mathutil.trilateration(self.anchors[:3], [sp*sp for sp in pos])
    def set_position(self, newpos, homing_axes):
        for s in self.steppers:
            s.set_position(newpos)
    def home(self, homing_state):
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.steppers, forcepos, self.home_position)
        homing_state.set_homed_position([0., 0., 0.])
    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True
    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            allowed_radius = self.radius - math.sqrt(
                self.min_arm2 - (self.min_arm_length - above_z_limit)**2
            )
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)
    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'cone_start_z': self.limit_z,
        }
    def get_calibration(self):
        endstops = [s.get_homing_info().position_endstop
                    for s in self.steppers]
        stepdists = [s.get_steppers()[0].get_step_dist()
                     for s in self.steppers]

def load_kinematics(toolhead, config):
    return WinchKinematics(toolhead, config)

