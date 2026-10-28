# -*- coding: utf-8 -*-
from client import GenParkEmbodiedGripper6DoFGraspStabilityEvaluator

def main():
    evaluator = GenParkEmbodiedGripper6DoFGraspStabilityEvaluator(friction_coeff=0.5)
    print("=== GenPark Physical AI 6-DoF Grasp Stability Evaluator ===")

    # Test 1: Ideal parallel antipodal grasp on a cylinder
    res1 = evaluator.evaluate_antipodal_grasp(
        contact_p1=(0.0, -0.04, 0.1),
        normal_1=(0.0, 1.0, 0.0),
        contact_p2=(0.0, 0.04, 0.1),
        normal_2=(0.0, -1.0, 0.0),
        com=(0.0, 0.0, 0.1)
    )
    print(f"Ideal Grasp: stable={res1['stable']}, score={res1['quality_score']}, jaw_width={res1['jaw_width_m']}m")

    # Test 2: Angled slippery grasp outside friction cone
    res2 = evaluator.evaluate_antipodal_grasp(
        contact_p1=(0.0, -0.04, 0.1),
        normal_1=(0.7071, 0.7071, 0.0),
        contact_p2=(0.0, 0.04, 0.1),
        normal_2=(-1.0, 0.0, 0.0),
        com=(0.0, 0.0, 0.1)
    )
    print(f"Slippery Grasp: stable={res2['stable']}, score={res2['quality_score']}, cone1={res2['contact1']['in_friction_cone']}")

if __name__ == "__main__":
    main()
