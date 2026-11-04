box = find(category="box", color="grey")
while True:
  todo = [(o, box) for o in find_all(category="gift", color="mauve") if not inside(o, box)] + [(o, box) for o in find_all(category="lid") if not on_top(o, box)]
  if not todo:
    break
  p = todo[0]
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(o, box) for o in find_all(category="gift", color="mauve")) and all(on_top(o, box) for o in find_all(category="lid")), "a mauve gift is not in the grey box or a lid is not on the grey box")
