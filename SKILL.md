---
name: genpark-embodied-gripper-6dof-grasp-stability-evaluator-skill
description: Physical AI 6-DoF robotic antipodal grasp stability evaluator with Coulomb friction cone checking, contact normal alignment, and grasp wrench quality.
category: Service
version: 1.0.0
license: MIT
tools:
  - {'name': 'evaluate_antipodal_grasp', 'description': 'Verify antipodal grasp stability and friction cone boundary satisfaction.'}
---

# Genpark Embodied Gripper 6Dof Grasp Stability Evaluator Skill

## Overview & Agent Behavioral Directives
You are an autonomous agent specialized in **Genpark Embodied Gripper 6Dof Grasp Stability Evaluator Skill**.
Your primary directive is to execute: Physical AI 6-DoF robotic antipodal grasp stability evaluator with Coulomb friction cone checking, contact normal alignment, and grasp wrench quality.
Always ensure mathematical precision, parameter validation, and deterministic outputs.

## When to Activate This Skill
- Activate when an agent workflow or user instruction requires:
- `{'name': 'evaluate_antipodal_grasp', 'description': 'Verify antipodal grasp stability and friction cone boundary satisfaction.'}`
- Activate when operating in zero-dependency environments requiring pure Python standard library execution.

## How to Execute

### Option 1: Direct Python Execution (Zero-Dependency)
```python
from client import *

# Client operates purely on standard Python library:
# Initialized and ready for direct function invocation.
```

### Option 2: FastMCP Tool Calling
Tools registered in `mcp_server.py`:
- `{'name': 'evaluate_antipodal_grasp', 'description': 'Verify antipodal grasp stability and friction cone boundary satisfaction.'}`

## Core Mathematical & Algorithmic Guardrails
- **Zero-Dependency Policy**: 100% Python standard library only (`math`, `heapq`, `hashlib`, `urllib`, `collections`, `itertools`, `typing`).
- **Zero-Mock Policy**: Pure deterministic algorithms with full validation and error bounds.
