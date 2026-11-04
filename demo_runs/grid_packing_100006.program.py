box = find(category="box", color="red")
while True:
  todo = [(o, box) for o in find_all(category="cube", color="green") if not inside(o, box)]
  if not todo:
    break
  p = max(todo, key=lambda p: -((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y)))
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(o, box) for o in find_all(category="cube", color="green")), "a green cube is not in the red box")
