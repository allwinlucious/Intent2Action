for color in ["red", "blue"]:
  box = find(category="box", color=color)
  for cube in find_all(category="cube", color=color):
    pick(cube)
    place(cube, box)
verify(lambda: all(inside(c, find(category="box", color=c.color)) for c in find_all(category="cube")
                   if c.color in ["red", "blue"]), "a cube is in the wrong box")
