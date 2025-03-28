from math import floor, acos, atan2, pi, cos, sin
from random import randint, random
from pygame import *

WIDTH, HEIGHT = 1710, 984
FPS = 60

init()
screen = display.set_mode((WIDTH, HEIGHT), RESIZABLE)
display.set_caption("john fish")
clock = time.Clock()

class Point:
    def __init__(self, pos: Vector2, radius: float):
        self.position = Vector2(pos)
        self.radius = radius
    
    def move(self, target: Vector2, value: float):
        self.position = Vector2.lerp(self.position, target, value)

class Fish:
    def __init__(self, scale: float, length: int = 17, speed: float = 2):
        self.scale = scale
        self.length = length
        self.speed = speed
        self.radii = [self.scale * (-0.5 * (i-2.7)**2 + 20) if i < 8 else self.scale * (15-i + 2) for i in range(self.length)]
        self.points = [Point(Vector2([-1 * i + WIDTH//2, HEIGHT//2]), self.radii[i]) for i in range(len(self.radii))]
        self.head = self.points[0]
        self.tail = self.points[-1]
    
    def move(self, target: Vector2, value: float):
        self.head.move(target, value)
    
    def update(self):
        for i in range(1, len(self.points)):
            p1 = self.points[i]
            p2 = self.points[i-1]
            
            if (p1.position - p2.position).magnitude() > p2.radius:
                p1.position = (p1.position - p2.position).normalize() * floor(p2.radius) + p2.position
            
            if i < 2:
                continue
            p3 = self.points[i-2]

            anchor_angle = atan2(p2.position.y - p3.position.y, p2.position.x - p3.position.x)
            current_angle = atan2(p1.position.y - p2.position.y, p1.position.x - p2.position.x)

            new_angle = constrain_angle(current_angle, anchor_angle, 0.5)
            dist = (p1.position - p2.position).magnitude()
            p1.position = p2.position + Vector2(cos(new_angle), sin(new_angle)) * dist

def constrain_angle(angle: float, anchor: float, constraint: float):
    diff = relative_angle_difference(angle, anchor)
    if abs(diff) <= constraint:
        return angle
    elif diff > 0:
        return anchor + constraint
    else:
        return anchor - constraint

def relative_angle_difference(angle: float, anchor: float):
    diff = (angle - anchor + pi) % (2*pi) - pi
    return diff

def simplify_angle(angle: float):
    return angle % (2*pi)

def draw_cubic_bezier(p0: Vector2, p1: Vector2, p2: Vector2, p3: Vector2, precision: int):
    draw_points = []
    for t in (i/precision for i in range(precision+1)):
        point = (1 - t)**3 * p0 + 3*(1 - t)**2 * t * p1 + 3*(1 - t)*t**2 * p2 + t**3 * p3
        draw_points.append(point)
    draw.lines(screen, (120, 120, 120), False, draw_points, 2)

def calculate_side_points(p1: Point, p2: Point):
    direction = (p2.position - p1.position).normalize()
    perpendicular = Vector2(-direction.y, direction.x)
    left = p1.position + perpendicular * p1.radius
    right = p1.position - perpendicular * p1.radius
    return left, right

def calculate_head_tail(fish: Fish):
    head, tail = fish.head, fish.tail
    head_direction = -1 * (fish.points[1].position - head.position).normalize()
    tail_direction = (fish.points[-2].position - tail.position).normalize()
    
    head_front = head.position + head_direction * head.radius
    tail_back = tail.position + tail_direction * tail.radius
    
    return head_front, tail_back

def draw_fish(fish: Fish):
    points = fish.points
    left_points = []
    right_points = []
    for i in range(len(points)-1):
        p1 = points[i]
        p2 = points[i+1]
        left, right = calculate_side_points(p1, p2)
        left_points.append(left)
        right_points.append(right)
    
    # body
    front, back = calculate_head_tail(fish)
    draw_points = [front] + left_points + [back] + right_points[::-1]
    num_points = len(draw_points)
    tension = 0.2
    precision = 20
    for i in range(num_points):
        p0 = draw_points[i]
        p3 = draw_points[(i + 1) % num_points]
        
        prev_p = draw_points[(i - 1) % num_points]
        next_p = draw_points[(i + 2) % num_points]
        
        c1 = p0 + (p3 - prev_p) * tension
        c2 = p3 - (next_p - p0) * tension
        
        draw_cubic_bezier(p0, c1, c2, p3, precision)

    # eyes
    head_direction = -1 * (points[1].position - fish.head.position).normalize()
    eyeline = Vector2(-head_direction.y, head_direction.x)

    draw.circle(screen, (255, 255, 255), fish.head.position + (eyeline * -(fish.scale * 40/3)), int(fish.scale * 5))
    draw.circle(screen, (255, 255, 255), fish.head.position + (eyeline * (fish.scale * 40/3)), int(fish.scale * 5))

sin_timer = 0
wiggle_amplitude = 200
wiggle_frequency = 2 * pi

small_fish = Fish(scale = 1.5, speed = 2)
aquarium = [small_fish]
target = Vector2(WIDTH // 2, HEIGHT // 2)
dvd = Vector2(WIDTH // 2, HEIGHT // 2)
dvd_direction = Vector2(random(), random()).normalize()
dvd_speed = 2 * small_fish.speed

follow_mouse = True
follow_target = False

while True:
    for e in event.get():
        if e.type == QUIT:
            quit()
        elif e.type == VIDEORESIZE:
            WIDTH, HEIGHT = e.size
            screen = display.set_mode((WIDTH, HEIGHT), RESIZABLE)
    
    screen.fill((20, 20, 20))

    for fish in aquarium:
        if aquarium.index(fish) == 0:
            difference = (mouse.get_pos() if follow_mouse else target if follow_target else dvd) - fish.head.position
        else:
            difference = aquarium[aquarium.index(fish)-1].head.position - fish.head.position
        if difference.magnitude() > 10:
            sin_timer += 1/FPS
            direction = difference.normalize()
            perp = Vector2(-direction.y, direction.x)
            
            sin_offset = perp * sin(sin_timer * wiggle_frequency) * wiggle_amplitude * (difference.magnitude()/400)
            target_position = difference + fish.head.position + sin_offset
            fish.move(target=target_position, value=min(0.9, (1/FPS) * fish.speed))
        elif fish == small_fish:
            target = Vector2(floor(random() * WIDTH), floor(random() * HEIGHT))
            # fish.speed += random() * 0.01 - 0.01

        fish.update()
        draw_fish(fish)

    if dvd.x >= WIDTH or dvd.x <= 0: dvd_direction.x *= -1
    if dvd.y >= HEIGHT or dvd.y <= 0: dvd_direction.y *= -1
    dvd += dvd_direction * dvd_speed

    display.flip()
    clock.tick(FPS)
