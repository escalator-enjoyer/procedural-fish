"""
john worm (chained constraints):
- pick anchor point
- scale difference betweeen target and anchor point down to be inside the acceptable distance
    (pick a radius (acceptable distance) to draw a circle, if target point outside circle, drag point back to the circle)
- can have a bunch of points following a head point with equal radii and head point is always lerping to the mouse
"""

from math import floor
from random import randint
from pygame import *

WIDTH, HEIGHT = 800, 800
FPS = 60

init()
screen = display.set_mode((WIDTH, HEIGHT))
display.set_caption("john worm")
clock = time.Clock()

class Point:
    def __init__(self, pos: Vector2, radius: float):
        self.position = Vector2(pos) # no idea why it keeps converting to list on init but whatever
        self.radius = radius
    
    def move(self, target: Vector2, value: float):
        self.position = Vector2.lerp(self.position, target, value)
    
def draw_worm(points: list[Point]):
    for p in points:
        draw.circle(screen, (150, 120, 120), p.position, p.radius, 3)


head = Point([WIDTH//2, HEIGHT//2], 10)

# point going from head to tail
points = [head] + [Point(Vector2([-10 * i + WIDTH//2, HEIGHT//2]), 10) for i in range(10)]

# main loop
while True:
    for e in event.get():
        if e.type == QUIT:
            quit()
    
    mouseX, mouseY = mouse.get_pos()

    # lerp head towards mouse
    points[0].move(target=Vector2(mouseX, mouseY), value=(1/FPS)*5)

    for i in range(1, len(points)):
        p = points[i]
        if (p.position - points[i-1].position).magnitude() > points[i-1].radius:
            # scale vector down to be on the circle
            p.position = (p.position - points[i-1].position).normalize() * floor(points[i-1].radius) + points[i-1].position

    screen.fill((20, 20, 20))
    draw_worm(points)
    display.flip()
    clock.tick(FPS)
