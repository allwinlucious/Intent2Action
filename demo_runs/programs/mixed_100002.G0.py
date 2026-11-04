red_box = find(category="box", color="red")
blue_box = find(category="box", color="blue")

for cube in find_all(category="cube", color="red"):
    pick(cube)
    place(cube, red_box)

for cube in find_all(category="cube", color="blue"):
    pick(cube)
    place(cube, blue_box)

report("red and blue cubes placed in matching boxes")

verify(lambda: all(inside(c, red_box) for c in find_all(category="cube", color="red")), "a red cube is not in the red box")
verify(lambda: all(inside(c, blue_box) for c in find_all(category="cube", color="blue")), "a blue cube is not in the blue box")
