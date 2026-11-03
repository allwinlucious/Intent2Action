"""The over-wide carton as a recovery case: what a model would do, robot calls recorded, not executed.

  ASK_BASE=<endpoint> ASK_KEY=<key> ASK_MODEL=<model> [HINT=1] .venv/bin/python -u tools/ask_push.py

Read-only tools get the scene data of tests/e2e/push_check.py; motion tools are answered "done (recorded only)".
HINT=1 adds one sentence that the arm can move objects by pushing (force-limited motion tools)."""
import json, os, sys
sys.path[:0] = [".", "gen"]
from program_runtime.agent import ChatClient, recovery_messages, tool_specs, _parse_decision
from mcp_server.server import DESCRIPTOR_SET, Tools

program = ('zone = find(category="zone", color="green")\ncarton = find(category="carton")\npick(carton)\n'
           'place(carton, zone)\nverify(lambda: on_top(find(category="carton"), zone), "the carton is not on the zone")\n')
objects = [
  {"id": "obj_35", "category": "table", "color": "brown", "size": "large", "x": 0.45, "y": 0.0, "z": -0.75, "yaw": 0.0, "sx": 1.2, "sy": 0.8, "sz": 0.75, "attached": False},
  {"id": "obj_33", "category": "carton", "color": "brown", "size": "large", "x": 0.5, "y": 0.22, "z": 0.0, "yaw": 0.0, "sx": 0.12, "sy": 0.12, "sz": 0.08, "attached": False},
  {"id": "obj_65", "category": "zone", "color": "green", "size": "large", "x": 0.5, "y": -0.15, "z": -0.0019, "yaw": 0.0, "sx": 0.16, "sy": 0.16, "sz": 0.002, "attached": False},
]
state = {"tool_pose": {"x": 0.307, "y": 0.0, "z": 0.49, "qx": -0.0002, "qy": 1.0, "qz": 0.0, "qw": 0.0}, "gripper_width": 0.08,
         "speed": 0.5, "faults": [], "note": "TCP is the fingertip centre; this orientation points the gripper straight down"}
context = {"program": "\n".join(f"{i+1}: {l}" for i, l in enumerate(program.splitlines())), "line": 3, "kind": "grasp",
           "message": "object wider than the gripper opening (0.075 m) in every horizontal direction", "call": "pick(obj_33)",
           "objects": objects, "state": state, "history": [], "budget": {"handoffs": 2, "patches": 1}}
messages = recovery_messages(context)
if os.environ.get("HINT") == "1":
  messages[0]["content"] += (" Robot tools can also change the world without a grasp: the arm can move an object by pushing it with the gripper (motion tools such as move_to_contact and move_compliant are force-limited). If your tool actions achieve what the failed step was for, decide skip.")
specs = tool_specs(Tools(DESCRIPTOR_SET))
client = ChatClient(base_url=os.environ.get("ASK_BASE", "https://api.meta.ai/v1"), api_key=os.environ.get("ASK_KEY") or os.environ["META_API_KEY"],
                    model=os.environ.get("ASK_MODEL", "muse-wonder-1.0"))
print(f"model {client.model} hint {os.environ.get('HINT') == '1'}")
calls_seen = []
for turn in range(14):
  message = client.chat(messages, tools=specs)
  calls = message.get("tool_calls") or []
  if not calls:
    print("\n=== FINAL DECISION ===\n", message.get("content")); print("parsed:", _parse_decision(message.get("content"))); break
  messages.append(message)
  for call in calls:
    name, args = call["function"]["name"], call["function"].get("arguments") or "{}"
    calls_seen.append((name, args))
    print(f"{len(calls_seen):2d}. {name}({args})")
    if name == "list_objects": result = json.dumps({"objects": objects})
    elif name == "get_state": result = json.dumps(state)
    else: result = "done (recorded only: not executed in this dry test)"
    messages.append({"role": "tool", "tool_call_id": call.get("id", ""), "content": result})
