box = find(category="box", color="red")
for cube in find_all(category="cube"):
    pick(cube)
    place(cube, box)
report(f"{len(find_all(category='cube'))} cubes placed in the red box")
verify(lambda: all(inside(c, box) for c in find_all(category="cube")), "a cube is not in the red box")
