reds = find_all(category="gift", color="red")
top = reds[0]
for r in reds[1:]:
  pick(r)
  place(r, top)
  top = r
box = find(category="box", color="grey")
gift = find(category="gift", color="mauve")
lid = find(category="lid")
green = find(category="gift", color="green")
pick(box)
place(box, green)
pick(gift)
place(gift, box)
pick(lid)
place(lid, box)
verify(lambda: on_top(box, green) and inside(gift, box) and on_top(lid, box), "not packed and stacked")
