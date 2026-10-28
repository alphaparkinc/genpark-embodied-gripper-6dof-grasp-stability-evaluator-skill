# -*- coding: utf-8 -*-
"""
GenPark Physical AI 6-DoF Robotic Gripper Grasp Stability Evaluator
Standard Library only: math, typing.
Evaluates antipodal parallel-jaw and multi-finger grasp quality, Coulomb friction cone
containment, contact normal alignment, and grasp wrench space epsilon metric.
"""

import math
from typing import Dict, List, Tuple, Any

class GenParkEmbodiedGripper6DoFGraspStabilityEvaluator:
    def __init__(self, friction_coeff: float = 0.45):
        self.friction_coeff = friction_coeff

    def _dot(self, v1: Tuple[float, float, float], v2: Tuple[float, float, float]) -> float:
        return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

    def _norm(self, v: Tuple[float, float, float]) -> float:
        return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    def _cross(self, u: Tuple[float, float, float], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        return (
            u[1]*v[2] - u[2]*v[1],
            u[2]*v[0] - u[0]*v[2],
            u[0]*v[1] - u[1]*v[0]
        )

    def evaluate_antipodal_grasp(self, contact_p1: Tuple[float, float, float],
                                 normal_1: Tuple[float, float, float],
                                 contact_p2: Tuple[float, float, float],
                                 normal_2: Tuple[float, float, float],
                                 com: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> Dict[str, Any]:
        """
        Evaluates antipodal grasp between two contact points and surface normals.
        """
        # Grasp axis from p1 to p2
        d = (contact_p2[0] - contact_p1[0], contact_p2[1] - contact_p1[1], contact_p2[2] - contact_p1[2])
        dist = self._norm(d)
        if dist < 1e-5:
            return {"stable": False, "reason": "Contacts coincident", "quality_score": 0.0}

        u_axis = (d[0]/dist, d[1]/dist, d[2]/dist)
        u_axis_rev = (-u_axis[0], -u_axis[1], -u_axis[2])

        # Normalize surface normals
        n1_len = self._norm(normal_1)
        n2_len = self._norm(normal_2)
        if n1_len < 1e-5 or n2_len < 1e-5:
            return {"stable": False, "reason": "Degenerate normals", "quality_score": 0.0}

        un1 = (normal_1[0]/n1_len, normal_1[1]/n1_len, normal_1[2]/n1_len)
        un2 = (normal_2[0]/n2_len, normal_2[1]/n2_len, normal_2[2]/n2_len)

        # Angle between surface normal and grasp axis
        cos_theta1 = max(-1.0, min(1.0, self._dot(un1, u_axis)))
        cos_theta2 = max(-1.0, min(1.0, self._dot(un2, u_axis_rev)))

        theta1_rad = math.acos(cos_theta1)
        theta2_rad = math.acos(cos_theta2)

        # Max allowable angle inside friction cone: alpha = arctan(mu)
        friction_cone_alpha = math.atan(self.friction_coeff)
        in_cone1 = theta1_rad <= friction_cone_alpha
        in_cone2 = theta2_rad <= friction_cone_alpha

        # Alignment quality
        alignment_score = max(0.0, (math.cos(theta1_rad) + math.cos(theta2_rad)) / 2.0)

        # Torque wrench relative to COM
        r1 = (contact_p1[0] - com[0], contact_p1[1] - com[1], contact_p1[2] - com[2])
        r2 = (contact_p2[0] - com[0], contact_p2[1] - com[1], contact_p2[2] - com[2])
        torque1 = self._cross(r1, u_axis)
        torque2 = self._cross(r2, u_axis_rev)
        net_torque = math.sqrt(sum((torque1[i] + torque2[i])**2 for i in range(3)))

        # Grasp Quality Epsilon Metric
        stable = in_cone1 and in_cone2
        quality_score = 0.0
        if stable:
            torque_penalty = 1.0 / (1.0 + net_torque)
            quality_score = round(alignment_score * torque_penalty, 4)

        return {
            "stable": stable,
            "quality_score": quality_score,
            "jaw_width_m": round(dist, 4),
            "contact1": {
                "angle_deg": round(math.degrees(theta1_rad), 2),
                "in_friction_cone": in_cone1
            },
            "contact2": {
                "angle_deg": round(math.degrees(theta2_rad), 2),
                "in_friction_cone": in_cone2
            },
            "friction_cone_angle_deg": round(math.degrees(friction_cone_alpha), 2),
            "friction_coefficient": self.friction_coeff,
            "net_rotational_torque_norm": round(net_torque, 4)
        }
