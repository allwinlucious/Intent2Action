red = find(category="cube", color="red")
yellow = find(category="cube", color="yellow")
pick(yellow)
place(yellow, red)
verify(lambda: on_top(find(category="cube", color="yellow"), find(category="cube", color="red")),
       "the yellow cube is not on the red cube")
