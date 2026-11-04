box = find(category="box", color="blue")
while True:
  todo = [(o, box) for o in find_all(category="cube") if not inside(o, box)]
  if not todo:
    break
  todo_objects = [p[0] for p in todo]
  cx = sum([o.x for o in todo_objects]) / len(todo_objects)
  cy = sum([o.y for o in todo_objects]) / len(todo_objects)
  p = max(todo, key=lambda p: (p[0].x - cx) * (p[0].x - cx) + (p[0].y - cy) * (p[0].y - cy))
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(o, box) for o in find_all(category="cube")), "a cube is not in the blue box")
