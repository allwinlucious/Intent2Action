blue_cube = find(category="cube", color="blue")
red_cube = find(category="cube", color="red")
while True:
  todo = [(o, blue_cube) for o in find_all(category="cube", color="red") if not on_top(o, blue_cube)] + [(o, red_cube) for o in find_all(category="cube", color="yellow") if not on_top(o, red_cube)]
  if not todo:
    break
  p = max(todo, key=lambda p: -((p[0].x - p[1].x) * (p[0].x - p[1].x) + (p[0].y - p[1].y) * (p[0].y - p[1].y)))
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(on_top(o, blue_cube) for o in find_all(category="cube", color="red")) and all(on_top(o, red_cube) for o in find_all(category="cube", color="yellow")), "a red cube is not on the blue cube or a yellow cube is not on the red cube")
