box = find(category="box", color="grey")
gift = find(category="gift", color="green")
while True:
  todo = [(o, box) for o in find_all(category="gift", color="mauve") if not inside(o, box)] + [(box, gift) if not on_top(box, gift) else None for _ in [0]] + [(o, box) for o in find_all(category="lid") if not on_top(o, box)]
  if not todo:
    break
  p = todo[0]
  if p is None:
    continue
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(inside(o, box) for o in find_all(category="gift", color="mauve")) and all(on_top(o, box) for o in find_all(category="lid")) and on_top(box, gift), "a mauve gift is not in the grey box or a lid is not on the grey box or the grey box is not on the green gift")
