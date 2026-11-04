box = find(category="box", color="blue")
for cube in find_all(category="cube"):
  pick(cube)
  place(cube, box)
verify(lambda: all(inside(c, find(category="box", color="blue")) for c in find_all(category="cube")), "a cube is not in the blue box")
