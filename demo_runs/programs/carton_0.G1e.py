zone = find(category="zone", color="green")
while True:
  todo = [(o, zone) for o in find_all(category="carton") if not on_top(o, zone)]
  if not todo:
    break
  p = todo[0]
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(on_top(o, zone) for o in find_all(category="carton")), "a carton is not on the green zone")
