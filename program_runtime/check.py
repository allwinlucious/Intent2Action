"""Static checks for programs executed by the program runtime."""

import ast
import re


API_FUNCTIONS = {
  "find", "find_all", "pick", "place", "held", "inside", "on_top", "verify", "report",
}
BUILTINS = {
  "len", "sorted", "min", "max", "sum", "range", "enumerate", "abs", "any", "all", "round",
  "list", "zip", "str",
}
OBJ_FIELDS = {
  "id", "category", "color", "size", "x", "y", "z", "yaw", "sx", "sy", "sz", "attached",
}
_ALLOWED = {
  ast.Module, ast.Expr, ast.Assign, ast.AugAssign, ast.For, ast.While, ast.If, ast.Break, ast.Continue,
  ast.Pass, ast.Name, ast.Load, ast.Store, ast.Constant, ast.Call, ast.keyword, ast.Attribute,
  ast.Subscript, ast.Slice, ast.Compare, ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE, ast.Is,
  ast.IsNot, ast.In, ast.NotIn, ast.BoolOp, ast.And, ast.Or, ast.UnaryOp, ast.Not, ast.USub, ast.BinOp,
  ast.Add, ast.Sub, ast.Mult, ast.Div, ast.Mod, ast.FloorDiv, ast.List, ast.Tuple, ast.Dict,
  ast.ListComp, ast.GeneratorExp, ast.comprehension, ast.IfExp, ast.JoinedStr, ast.FormattedValue,
  ast.Lambda, ast.arguments, ast.arg,
}


def _call_name(node: ast.AST) -> str | None:
  return node.id if isinstance(node, ast.Name) else None


def _is_report(statement: ast.stmt) -> bool:
  return (
      isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call)
      and _call_name(statement.value.func) == "report"
  )


def _is_verify(statement: ast.stmt) -> bool:
  return (
      isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call)
      and _call_name(statement.value.func) == "verify"
  )


def check(source: str, *, patch: bool = False) -> list[str]:
  """Returns policy problems in *source*; an empty list means it is safe to run."""
  del patch
  try:
    tree = ast.parse(source)
  except SyntaxError as error:
    return [f"syntax error line {error.lineno}: {error.msg}"]

  problems = []
  for node in ast.walk(tree):
    line = getattr(node, "lineno", 0)
    if type(node) not in _ALLOWED:
      problems.append(f"line {line}: {type(node).__name__} is not allowed")
      continue
    if isinstance(node, ast.Name) and node.id.startswith("_"):
      problems.append(f"line {line}: names starting with _ are not allowed")
    if isinstance(node, ast.Call):
      name = _call_name(node.func)
      if name not in API_FUNCTIONS | BUILTINS:
        problems.append(f"line {line}: call must use an allowed function")
      if name == "verify" and (not node.args or not isinstance(node.args[0], ast.Lambda)):
        problems.append(f"line {line}: verify first argument must be a lambda")
      elif name == "verify":
        arguments = node.args[0].args
        if arguments.posonlyargs or arguments.args or arguments.kwonlyargs or arguments.vararg or arguments.kwarg:
          problems.append(f"line {line}: verify lambda must take no arguments")
    if isinstance(node, ast.Attribute) and node.attr not in OBJ_FIELDS:
      problems.append(f"line {line}: object attribute {node.attr!r} is not allowed")
    if isinstance(node, ast.Constant):
      if isinstance(node.value, str) and "obj_" in node.value:
        problems.append(f"line {line}: hard-coded object id")
      if isinstance(node.value, float):
        problems.append(f"line {line}: hard-coded number; resolve objects with find")

  statements = [statement for statement in tree.body if not _is_report(statement)]
  if not statements or not _is_verify(statements[-1]):
    problems.append("the program must end with verify")
  return problems
