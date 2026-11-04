red_box = find(category="box", color="red")
blue_box = find(category="box", color="blue")
while True:
  todo = [(o, red_box) for o in find_all(category="cube", color="red") if not inside(o, red_box)] + [(o, blue_box) for o in find_all(category="cube", color="blue") if not inside(o, blue_box)]
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
verify(lambda: all(inside(o, red_box) for o in find_all(category="cube", color="red")) and all(inside(o, blue_box) for o in find_all(category="cube", color="blue")), "a red cube is not in the red box or a blue cube is not in the blue box")
