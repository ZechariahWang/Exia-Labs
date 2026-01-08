import math
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class MotionPrimitive:
    dx: float
    dy: float
    dtheta: float
    arc_length: float
    steering_angle: float
    cost: float


class MotionPrimitiveSet:
    def __init__(
        self,
        wheelbase: float = 1.3,
        max_steering: float = 0.6,
        step_length: float = 0.3,
        num_angles: int = 7,
    ):
        self.wheelbase = wheelbase
        self.max_steering = max_steering
        self.step_length = step_length
        self.num_angles = num_angles

        self.min_turn_radius = wheelbase / math.tan(max_steering)

        self.primitives: List[MotionPrimitive] = []
        self._generate_primitives()

    def _generate_primitives(self):
        steering_angles = np.linspace(-self.max_steering, self.max_steering, self.num_angles)

        for steer in steering_angles:
            if abs(steer) < 0.01:
                dx = self.step_length
                dy = 0.0
                dtheta = 0.0
                arc_length = self.step_length
            else:
                R = self.wheelbase / math.tan(abs(steer))
                arc_angle = self.step_length / R

                dx = R * math.sin(arc_angle)
                dy = R * (1.0 - math.cos(arc_angle)) * np.sign(steer)
                dtheta = arc_angle * np.sign(steer)
                arc_length = abs(R * arc_angle)

            cost = 1.0 + 0.5 * abs(steer) / self.max_steering

            self.primitives.append(MotionPrimitive(
                dx=dx,
                dy=dy,
                dtheta=dtheta,
                arc_length=arc_length,
                steering_angle=steer,
                cost=cost
            ))

        for steer in steering_angles:
            if abs(steer) < 0.01:
                dx = -self.step_length
                dy = 0.0
                dtheta = 0.0
                arc_length = self.step_length
            else:
                R = self.wheelbase / math.tan(abs(steer))
                arc_angle = self.step_length / R

                dx = -R * math.sin(arc_angle)
                dy = -R * (1.0 - math.cos(arc_angle)) * np.sign(steer)
                dtheta = -arc_angle * np.sign(steer)
                arc_length = abs(R * arc_angle)

            cost = 1.5 + 0.5 * abs(steer) / self.max_steering

            self.primitives.append(MotionPrimitive(
                dx=dx,
                dy=dy,
                dtheta=dtheta,
                arc_length=arc_length,
                steering_angle=steer,
                cost=cost
            ))

    def get_successors(
        self,
        x: float,
        y: float,
        theta: float
    ) -> List[Tuple[float, float, float, float, float]]:
        successors = []

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        for prim in self.primitives:
            world_dx = prim.dx * cos_theta - prim.dy * sin_theta
            world_dy = prim.dx * sin_theta + prim.dy * cos_theta

            new_x = x + world_dx
            new_y = y + world_dy
            new_theta = self._normalize_angle(theta + prim.dtheta)

            successors.append((
                new_x,
                new_y,
                new_theta,
                prim.arc_length,
                prim.cost
            ))

        return successors

    def interpolate_arc(
        self,
        x: float,
        y: float,
        theta: float,
        primitive_idx: int,
        num_samples: int = 5
    ) -> List[Tuple[float, float, float]]:
        if primitive_idx >= len(self.primitives):
            return [(x, y, theta)]

        prim = self.primitives[primitive_idx]
        samples = []

        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        for t in np.linspace(0, 1, num_samples):
            if abs(prim.steering_angle) < 0.01:
                local_dx = prim.dx * t
                local_dy = 0.0
                local_dtheta = 0.0
            else:
                R = self.wheelbase / math.tan(abs(prim.steering_angle))
                arc_angle = (self.step_length / R) * t

                sign = np.sign(prim.steering_angle)
                if prim.dx > 0:
                    local_dx = R * math.sin(arc_angle)
                    local_dy = R * (1.0 - math.cos(arc_angle)) * sign
                    local_dtheta = arc_angle * sign
                else:
                    local_dx = -R * math.sin(arc_angle)
                    local_dy = -R * (1.0 - math.cos(arc_angle)) * sign
                    local_dtheta = -arc_angle * sign

            world_dx = local_dx * cos_theta - local_dy * sin_theta
            world_dy = local_dx * sin_theta + local_dy * cos_theta

            sample_x = x + world_dx
            sample_y = y + world_dy
            sample_theta = self._normalize_angle(theta + local_dtheta)

            samples.append((sample_x, sample_y, sample_theta))

        return samples

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
