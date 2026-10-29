"""Stage-1 data: single calls (valid and systematically wrong) sent to the real stack, replies recorded.

  .venv-ml/bin/python learn/stage1.py --intents 2500 --seed 1 --out stage1_train
  .venv-ml/bin/python learn/stage1.py --intents 300 --seed 2 --out stage1_bench

Per situation (random scene, sometimes holding a cube, sometimes faulted by nothing — the stack decides):
sample an intent (natural-language instruction + the call that does it + wrong variants), send one wrong
variant (if any) then the right call, record both with the stack's status and message. One JSONL line per
intent in logs/learn/<out>.jsonl:
  {intent, family, situation: {objects, state}, valid: {name, arguments, status, result},
   wrong: {mutation, name, arguments, status, result} | null, infeasible}
"""

import argparse
import json
import math
import random
import time

import agent
import env

DOWN = {"qx": 0.0, "qy": 1.0, "qz": 0.0, "qw": 0.0}
SETUP = [None]  # the set-up move of the contact intent (not recorded): 2-5 cm above the table
INTENTS_PER_SCENE = 10
DESC = {("cube", "red"): "red cube", ("cube", "blue"): "blue cube", ("cube", "green"): "green cube",
        ("cube", "yellow"): "small yellow cube", ("plate", "white"): "plate", ("peg", "blue"): "peg",
        ("block_with_hole", "grey"): "grey block"}


def r3(v: float) -> float:
  return round(v, 3)


def desc(o: dict) -> str:
  return DESC.get((o["category"], o["color"]), f"{o['color']} {o['category']}")


def pose(x, y, z, q=DOWN) -> dict:
  return {"x": r3(x), "y": r3(y), "z": r3(z), **q}


def mm(p: dict) -> dict:
  return {**p, "x": r3(p["x"] * 1000), "y": r3(p["y"] * 1000), "z": r3(p["z"] * 1000)}


def nested(p: dict) -> dict:
  return {"position": {k: p[k] for k in "xyz"}, "orientation": {k: p[k] for k in ("qx", "qy", "qz", "qw")}}


def intents(rng: random.Random, objects: list[dict], state: dict) -> list:
  """Every applicable (family, text, valid call, [(mutation, wrong call)], infeasible) in this situation."""
  movable = [o for o in objects if o["category"] in ("cube", "peg") and not o.get("attached_to")]
  cubes = [o for o in movable if o["category"] == "cube"]
  plate = next((o for o in objects if o["category"] == "plate"), None)
  tcp = state.get("tool_pose") or {}
  joints = (state.get("joints") or {}).get("positions") or []
  holding = state.get("gripping")
  out = []
  if state.get("faulted") or not state.get("enabled", True):
    return [("clear_faults", "The robot reports a fault and is not enabled. Clear it so it can move again.", ("clear_faults", {}),
             [("unknown_tool", ("reset_faults", {}))], False)]
  if cubes:
    o = rng.choice(cubes)
    h = rng.choice([0.10, 0.15, 0.20])
    p = pose(o["pose"]["x"], o["pose"]["y"], o["pose"]["z"] + o["size_z"] + h)
    out.append(("move_above", f"Move the TCP to {h * 100:.0f} cm above the top of the {desc(o)} ({o['id']}), "
                f"gripper pointing down.", ("move_to_pose", {"pose": p}),
                [("millimetres", ("move_to_pose", {"pose": mm(p)})), ("nested_pose", ("move_to_pose", {"pose": nested(p)})),
                 ("zero_quaternion", ("move_to_pose", {"pose": {**p, "qy": 0.0}})),
                 ("bad_enum", ("move_to_pose", {"pose": p, "motion": "STRAIGHT"}))], False))
    out.append(("plan_grasp", f"Plan a grasp for the {desc(o)}.", ("plan_grasp", {"object_id": o["id"]}),
                [("name_not_id", ("plan_grasp", {"object_id": desc(o).replace(" ", "_")})),
                 ("wrong_key", ("plan_grasp", {"id": o["id"]})),
                 ("unknown_id", ("plan_grasp", {"object_id": f"obj_{rng.choice([n for n in range(100) if f'obj_{n:02d}' not in {x['id'] for x in objects}]):02d}"}))],
                False))
  if plate is not None and rng.random() < 0.3:
    out.append(("grasp_plate", "Plan a grasp for the plate.", ("plan_grasp", {"object_id": plate["id"]}), [], True))
  if tcp:
    x, y, z = (rng.uniform(0.3, 0.65), rng.uniform(-0.3, 0.3), rng.uniform(0.12, 0.4))
    p = pose(x, y, z)
    out.append(("move_xyz", f"Move the TCP in a straight line to x={x:.3f}, y={y:.3f}, z={z:.3f} m, pointing down.",
                ("move_to_pose", {"pose": p, "motion": "LINEAR"}),
                [("millimetres", ("move_to_pose", {"pose": mm(p), "motion": "LINEAR"})),
                 ("lowercase_enum", ("move_to_pose", {"pose": p, "motion": "linear"})),
                 ("unknown_tool", ("move_linear", {"pose": p}))], False))
    far = rng.uniform(1.1, 1.6)
    out.append(("unreachable", f"Move the TCP to x={far:.2f}, y=0.0, z=0.3 m, pointing down.",
                ("move_to_pose", {"pose": pose(far, 0.0, 0.3)}), [], True))
    out.append(("below_table", f"Move the TCP to x={x:.3f}, y={y:.3f}, z=-0.050 m.",
                ("move_to_pose", {"pose": pose(x, y, -0.05)}), [], True))
    out.append(("ik", f"Compute the joint angles that put the TCP at x={x:.3f}, y={y:.3f}, z={z:.3f} m, pointing down. Do not move.",
                ("compute_ik", {"pose": p}),
                [("millimetres", ("compute_ik", {"pose": mm(p)})), ("unknown_tool", ("inverse_kinematics", {"pose": p}))], False))
    out.append(("collision", f"Check whether the TCP at x={x:.3f}, y={y:.3f}, z={z:.3f} m pointing down would collide with anything.",
                ("check_collisions", {"pose": p}),
                [("both_oneof", ("check_collisions", {"pose": p, "joints": {"positions": joints}}))], False))
    above = rng.uniform(0.02, 0.05)
    SETUP[0] = ("move_to_pose", {"pose": pose(tcp["x"], tcp["y"], above + (0.05 if holding else 0.0)), "motion": "LINEAR"})
    force = rng.choice([5.0, 10.0, 15.0, 20.0])
    speed = r3(rng.uniform(0.0003, 0.0012) * force)
    out.append(("contact", f"Move the TCP straight down until it touches something, pressing with at most {force:g} N.",
                ("move_to_contact", {"dx": 0.0, "dy": 0.0, "dz": -1.0, "max_force": force, "speed": speed}),
                [("too_fast", ("move_to_contact", {"dx": 0.0, "dy": 0.0, "dz": -1.0, "max_force": force, "speed": r3(0.01 * force)})),
                 ("no_direction", ("move_to_contact", {"max_force": force, "speed": speed})),
                 ("upward", ("move_to_contact", {"dx": 0.0, "dy": 0.0, "dz": 1.0, "max_force": force, "speed": speed})),
                 ("missing_speed", ("move_to_contact", {"dx": 0.0, "dy": 0.0, "dz": -1.0, "max_force": force}))], False))
  if joints:
    j = [r3(q + rng.uniform(-0.15, 0.15)) for q in joints]
    out.append(("move_joints", f"Move the arm to the joint positions {j} (radians).", ("move_joints", {"joints": {"positions": j}}),
                [("degrees", ("move_joints", {"joints": {"positions": [r3(math.degrees(q)) for q in j]}})),
                 ("six_joints", ("move_joints", {"joints": {"positions": j[:6]}})),
                 ("flat_list", ("move_joints", {"joints": j}))], False))
    out.append(("fk", f"Which TCP pose do the joint positions {j} give? Do not move.", ("compute_fk", {"joints": {"positions": j}}),
                [("degrees_fk", ("compute_fk", {"joints": {"positions": [r3(math.degrees(q)) for q in j]}}))], False))
    jn, v, t = rng.randint(1, 7), rng.choice([-0.2, -0.1, 0.1, 0.2]), rng.choice([0.5, 1.0])
    out.append(("jog", f"Jog joint {jn} at {v:+g} rad/s for {t:g} s.", ("jog", {"joint": jn, "velocity": v, "duration_s": t}),
                [("zero_based", ("jog", {"joint": jn - 1, "velocity": v, "duration_s": t})),
                 ("deg_per_s", ("jog", {"joint": jn, "velocity": r3(math.degrees(v)), "duration_s": t}))], False))
  pct = rng.choice([30, 50, 70, 100])
  out.append(("speed", f"Set the robot's speed to {pct}% of its maximum.", ("set_speed", {"factor": pct / 100}),
              [("percent", ("set_speed", {"factor": float(pct)})), ("zero", ("set_speed", {"factor": 0.0}))], False))
  out.append(("home", "Move the robot back to its home position.", ("home", {}), [("unknown_tool", ("go_home", {}))], False))
  out.append(("state", "What is the robot's current state?", ("get_state", {}), [("unknown_tool", ("get_robot_state", {}))], False))
  out.append(("open", "Open the gripper.", ("release", {}), [("unknown_tool", ("open_gripper", {}))], False))
  if holding:
    held = next((o for o in objects if o.get("attached_to")), None)
    if held:
      out.append(("detach", f"The {desc(held)} has been released; remove it from the gripper in the world model.",
                  ("detach_object", {"id": held["id"]}),
                  [("wrong_key", ("detach_object", {"object_id": held["id"]})),
                   ("name_not_id", ("detach_object", {"id": desc(held).replace(" ", "_")}))], False))
  elif cubes:
    o = rng.choice(cubes)
    out.append(("attach", f"The gripper is holding the {desc(o)} now; attach it to the gripper in the world model.",
                ("attach_object", {"id": o["id"]}),
                [("wrong_key", ("attach_object", {"object_id": o["id"]})),
                 ("name_not_id", ("attach_object", {"id": desc(o).replace(" ", "_")}))], False))
  m = rng.choice([0.05, 0.1, 0.2, 0.5])
  out.append(("payload", f"Set the payload to {m:g} kg.", ("set_payload", {"mass": m}),
              [("grams", ("set_payload", {"mass": m * 1000})), ("negative", ("set_payload", {"mass": -m}))], False))
  return out


def observe() -> dict:
  return {name: json.loads(agent.call(name, {})[1]) for name in ("list_objects", "get_state")}


def send(name: str, arguments) -> dict:
  status, result = agent.call(name, arguments)
  return {"name": name, "arguments": arguments, "status": status, "result": result}


def holding_start(rng: random.Random, objects: list[dict]) -> None:
  """Picks a random cube (calls not recorded), so some situations have a held object."""
  cube = rng.choice([o for o in objects if o["category"] == "cube"])
  status, text = agent.call("plan_grasp", {"object_id": cube["id"]})
  if status != "OK":
    return
  plan = json.loads(text)
  for name, args in (("move_to_pose", {"pose": plan["pregrasp"]}), ("move_to_pose", {"pose": plan["grasp"], "motion": "LINEAR"}),
                     ("grasp", {}), ("attach_object", {"id": cube["id"]}),
                     ("move_to_pose", {"pose": {**plan["grasp"], "z": plan["grasp"]["z"] + 0.1}, "motion": "LINEAR"})):
    if agent.call(name, args)[0] != "OK":
      return


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--intents", type=int, required=True)
  parser.add_argument("--seed", type=int, required=True)
  parser.add_argument("--out", required=True)
  args = parser.parse_args()
  path = env.LOG_DIR / f"{args.out}.jsonl"
  done = sum(1 for _ in open(path)) if path.exists() else 0  # resume after a stop
  rng = random.Random(args.seed * 100_000 + done)
  start = time.monotonic()
  if done >= args.intents:
    print(f"DONE {done} intents (already)", flush=True)
    return
  with env.Stack(args.out), open(env.LOG_DIR / f"{args.out}.jsonl", "a") as out:
    while done < args.intents:
      names = sorted(set(rng.sample(env.CUBES, rng.randint(1, 4))) | {"plate", "peg"})
      env.new_scene(rng, names)
      agent.call("set_speed", {"factor": 1.0})
      if rng.random() < 0.3:
        holding_start(rng, observe()["list_objects"]["objects"])
      for _ in range(INTENTS_PER_SCENE):
        situation = observe()
        options = intents(rng, situation["list_objects"]["objects"], situation["get_state"])
        family, text, (name, arguments), wrongs, infeasible = rng.choice(options)
        if family == "contact":
          send(*SETUP[0])
          situation = observe()
        wrong = None
        if wrongs and rng.random() < 0.7:
          mutation, (wname, wargs) = rng.choice(wrongs)
          wrong = {"mutation": mutation, **send(wname, wargs)}
        valid = send(name, arguments)
        out.write(json.dumps({"intent": text, "family": family, "situation": situation, "valid": valid,
                              "wrong": wrong, "infeasible": infeasible}) + "\n")
        out.flush()
        done += 1
        print(f"{done} {family}: {'wrong ' + wrong['mutation'] + '=' + wrong['status'] + ', ' if wrong else ''}"
              f"valid={valid['status']} {valid['result'][:100] if valid['status'] != 'OK' else ''}", flush=True)
        if done >= args.intents:
          break
  print(f"DONE {done} intents in {(time.monotonic() - start) / 60:.0f} min", flush=True)


if __name__ == "__main__":
  main()
