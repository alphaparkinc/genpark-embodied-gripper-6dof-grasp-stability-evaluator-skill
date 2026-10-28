# -*- coding: utf-8 -*-
import sys, json
from client import GenParkEmbodiedGripper6DoFGraspStabilityEvaluator

evaluator = GenParkEmbodiedGripper6DoFGraspStabilityEvaluator()

def run_tool(name, args):
    if name == "evaluate_antipodal_grasp":
        p1 = tuple(args.get("contact_p1", [0.0, 0.0, 0.0]))
        n1 = tuple(args.get("normal_1", [1.0, 0.0, 0.0]))
        p2 = tuple(args.get("contact_p2", [0.1, 0.0, 0.0]))
        n2 = tuple(args.get("normal_2", [-1.0, 0.0, 0.0]))
        com = tuple(args.get("com", [0.05, 0.0, 0.0]))
        return evaluator.evaluate_antipodal_grasp(p1, n1, p2, n2, com)
    else:
        raise ValueError(f"Unknown tool: {name}")

def main():
    if "--test" in sys.argv:
        print("[TEST] Running GenPark Gripper 6-DoF Grasp Stability Evaluator Self-Test...")
        res = evaluator.evaluate_antipodal_grasp((0,0,0), (1,0,0), (0.08, 0, 0), (-1,0,0), (0.04, 0, 0))
        assert res["stable"] is True, "Antipodal grasp should be stable"
        assert res["quality_score"] > 0.8, "Quality score should be high"
        print("[TEST] All tests passed successfully!")
        return

    for line in sys.stdin:
        if not line.strip(): continue
        try:
            req = json.loads(line)
            res = run_tool(req.get("name"), req.get("arguments", {}))
            print(json.dumps({"jsonrpc": "2.0", "id": req.get("id"), "result": res}))
        except Exception as e:
            print(json.dumps({"jsonrpc": "2.0", "id": None, "error": {"code": -32603, "message": str(e)}}))
        sys.stdout.flush()

if __name__ == "__main__":
    main()
