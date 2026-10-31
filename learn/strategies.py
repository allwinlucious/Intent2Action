"""Strategy programs: the search expert's candidates, built from an intent's rules.

A strategy only decides the order of the objects to move. Every program re-reads the scene each iteration and acts
only on objects not yet satisfying their rule, so it is idempotent: it also works as a patch after a partial run.
"""

import json

STRATEGIES = ("listed", "most_free_first", "outer_first", "nearest_target_first", "top_first")
WORD_ORDER = ("size", "color", "category")
RELATION_CHECK = {"in": "inside", "on": "on_top"}


def _arguments(filters: dict) -> str:
  return ", ".join(f"{key}={json.dumps(value)}" for key, value in filters.items())


def _words(filters: dict) -> list[str]:
  return [filters[key] for key in WORD_ORDER if key in filters]


def _target_names(rules: list[dict]) -> list[tuple[dict, str]]:
  """One variable per distinct target: its category when there is one target, else all its filter words."""
  targets = []
  for rule in rules:
    if rule["target"] not in targets:
      targets.append(rule["target"])
  if len(targets) == 1:
    names = [targets[0].get("category", "target")]
  else:
    names = ["_".join(_words(target)) for target in targets]
  return list(zip(targets, names))


def _distance(a: str, b: str) -> str:
  return f"({a}.x - {b}.x) * ({a}.x - {b}.x) + ({a}.y - {b}.y) * ({a}.y - {b}.y)"


def _choice(strategy: str) -> list[str]:
  """Lines choosing the pair p from the non-empty work list todo."""
  if strategy == "listed":
    return ["p = todo[0]"]
  if strategy == "top_first":
    return ["p = max(todo, key=lambda p: p[0].z)"]
  if strategy == "nearest_target_first":
    return [f"p = max(todo, key=lambda p: -({_distance('p[0]', 'p[1]')}))"]
  centroid = [
    "todo_objects = [p[0] for p in todo]",
    "cx = sum([o.x for o in todo_objects]) / len(todo_objects)",
    "cy = sum([o.y for o in todo_objects]) / len(todo_objects)",
  ]
  from_centroid = "(p[0].x - cx) * (p[0].x - cx) + (p[0].y - cy) * (p[0].y - cy)"
  if strategy == "outer_first":
    return [*centroid, f"p = max(todo, key=lambda p: {from_centroid})"]
  if strategy == "most_free_first":
    # Missing neighbours count sx * sx * 100 each, so a lone object is the freest.
    return [*centroid,
            f"p = max(todo, key=lambda p: (sum(sorted([{_distance('p[0]', 'o')} for o in todo_objects if o != p[0]]",
            "                                         + [p[0].sx * p[0].sx * 100] * 2)[:2]),",
            f"                             {from_centroid}))"]
  raise ValueError(f"unknown strategy {strategy!r}")


def _pairs(rules: list[dict], names: dict) -> str:
  parts = []
  for rule in rules:
    target = names[json.dumps(rule["target"], sort_keys=True)]
    check = RELATION_CHECK[rule["relation"]]
    parts.append(f"[(o, {target}) for o in find_all({_arguments(rule['select'])}) if not {check}(o, {target})]")
  return " + ".join(parts)


def _message(rules: list[dict]) -> str:
  parts = []
  for rule in rules:
    noun = " ".join(_words(rule["select"]) + ([] if "category" in rule["select"] else ["object"]))
    article = "an" if noun[0] in "aeiou" else "a"
    preposition = "in" if rule["relation"] == "in" else "on"
    parts.append(f"{article} {noun} is not {preposition} the {' '.join(_words(rule['target']))}")
  return " or ".join(parts)


def program(intent: dict, strategy: str) -> str:
  """The strategy's program for the intent. Rules with relation "on" (towers) are moved in their listed order
  before any strategy choice: the intent gives the order of the levels."""
  rules = intent["rules"]
  targets = _target_names(rules)
  names = {json.dumps(target, sort_keys=True): name for target, name in targets}
  stack = [rule for rule in rules if rule["relation"] == "on"]
  loose = [rule for rule in rules if rule["relation"] == "in"]
  lines = [f"{name} = find({_arguments(target)})" for target, name in targets]
  lines.append("while True:")
  body = []
  if stack and loose:
    body += [f"stack = {_pairs(stack, names)}", f"todo = {_pairs(loose, names)}",
             "if not stack and not todo:", "  break", "if stack:", "  p = stack[0]", "else:",
             *(f"  {line}" for line in _choice(strategy))]
  else:
    body += [f"todo = {_pairs(stack or loose, names)}", "if not todo:", "  break",
             *(_choice("listed") if stack else _choice(strategy))]
  # An object already in the gripper (a pick that failed after grasping, then a patch) is placed first.
  work = "stack + todo" if stack and loose else "todo"
  body += [f"held_now = [q for q in {work} if q[0].attached]", "if held_now:", "  p = held_now[0]",
           "if not p[0].attached:", "  pick(p[0])", "place(p[0], p[1])"]
  lines += [f"  {line}" for line in body]
  checks = []
  for rule in rules:
    target = names[json.dumps(rule["target"], sort_keys=True)]
    check = RELATION_CHECK[rule["relation"]]
    checks.append(f"all({check}(o, {target}) for o in find_all({_arguments(rule['select'])}))")
  lines.append(f"verify(lambda: {' and '.join(checks)}, {json.dumps(_message(rules))})")
  return "\n".join(lines) + "\n"
