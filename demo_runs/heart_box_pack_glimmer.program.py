box = find(category="box")
while True:
  todo = [(o, box) for o in find_all(category="gift", color="mauve", size="small") if not inside(o, box)]
  if not todo:
    break
  p = max(todo, key=lambda p: -((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y)))
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(o, box) for o in find_all(category="gift", color="mauve", size="small")), "a mauve small gift is not in the box")
while True:
  todo = [(o, box) for o in find_all(category="lid") if not on_top(o, box)]
  if not todo:
    break
  p = max(todo, key=lambda p: -((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y)))
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(on_top(o, box) for o in find_all(category="lid")), "a lid is not on the box")
