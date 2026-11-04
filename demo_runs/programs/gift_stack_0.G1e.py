red_gift = find(category="gift", color="red")
green_gift = find(category="gift", color="green")
grey_box = find(category="box", color="grey")
while True:
  todo = [(o, green_gift) for o in find_all(category="box", color="grey") if not on_top(o, green_gift)] + [(o, grey_box) for o in find_all(category="lid") if not on_top(o, grey_box)] + [(o, grey_box) for o in find_all(category="gift", color="mauve") if not inside(o, grey_box)] + [(o, red_gift) for o in find_all(category="gift", color="red") if o != red_gift and not on_top(o, red_gift)]
  if not todo:
    break
  p = todo[0]
  held_now = [q for q in todo if q[0].attached]
  if held_now:
    p = held_now[0]
  if not p[0].attached:
    pick(p[0])
  place(p[0], p[1])
verify(lambda: all(on_top(o, green_gift) for o in find_all(category="box", color="grey")) and all(on_top(o, grey_box) for o in find_all(category="lid")) and all(inside(o, grey_box) for o in find_all(category="gift", color="mauve")) and all(on_top(o, red_gift) for o in find_all(category="gift", color="red") if o != red_gift), "the grey box is not on the green gift or the lid is not on the grey box or the mauve gift is not in the grey box or the red gifts are not stacked")
