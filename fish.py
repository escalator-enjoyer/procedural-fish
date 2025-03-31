from math import atan2, pi, cos, sin
from random import randint, random, uniform
from pygame import *

WIDTH, HEIGHT = 1710, 984
FPS = 60

FOOD_RADIUS = 16
food_list = []
last_food_regen = time.get_ticks()

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
    def __init__(self, scale: float):
        self.scale = scale
        self.min_scale = 0.75
        self.max_scale = 2.5
        self.length = 17
        self.radii = [self.scale * (-0.5 * (i-2.7)**2 + 20) if i < 8 else self.scale * max(2, (15-i + 2)) for i in range(17)]
        self.points = [Point(Vector2([-1 * i + WIDTH//2, HEIGHT//2]), self.radii[i]) for i in range(len(self.radii))]
        self.head = self.points[0]
        self.tail = self.points[-1]
        self.velocity = Vector2(0, 0)
        self.acceleration = Vector2(0, 0)
        self.max_force = 0.1
        self.wiggle_phase = random() * 2 * pi
        self.base_wiggle_freq = uniform(1.0, 2.5)
        self.wiggle_freq = self.base_wiggle_freq
        self.thrust_strength = 0.25 * scale
        self.target = None
        self.wander_angle = random() * 2 * pi
        self.max_speed = 50 / self.scale
        self.min_speed = 0.05
        self.last_out_time = 0
        self.out_of_water = False

    def apply_force(self, force):
        self.acceleration += force

    def seek(self, target):
        if (target - self.head.position).magnitude() == 0: self.head.position += Vector2(uniform(-0.1, 0.1), uniform(-0.1, 0.1))
        desired_direction = (target - self.head.position).normalize()
        if self.velocity.length() > 0:
            current_direction = self.velocity.normalize()
        else:
            current_direction = Vector2(1, 0)
        steering = (desired_direction - current_direction) * 0.5
        if steering.length() > self.max_force:
            steering.scale_to_length(self.max_force)
        self.apply_force(steering)
        self.wiggle_freq = self.base_wiggle_freq * 2

    def flee(self, threat):
        if (self.head.position - threat).magnitude() == 0: self.head.position += Vector2(random(), random())
        desired_direction = (self.head.position - threat).normalize()
        if self.velocity.length() > 0:
            current_direction = self.velocity.normalize()
        else:
            current_direction = Vector2(1, 0)
        steering = (desired_direction - current_direction) * 1.0
        if steering.length() > self.max_force:
            steering.scale_to_length(self.max_force)
        self.apply_force(steering)
        self.wiggle_freq = self.base_wiggle_freq * 3
    
    def wander(self):
        circle_dist = 100
        circle_rad = 50
        self.wander_angle += uniform(-0.5, 0.5)
        if self.velocity.magnitude() == 0: self.velocity = Vector2(0.01, 0)
        target = self.velocity.normalize() * circle_dist + self.head.position
        wander_offset = Vector2(cos(self.wander_angle)*circle_rad, sin(self.wander_angle)*circle_rad)
        self.seek(target + wander_offset)

    def update(self):
        # thrust based on orientation
        if len(self.points) > 1:
            direction = (self.points[0].position - self.points[1].position).normalize()
        else:
            direction = Vector2(1, 0)
        thrust = direction * (self.thrust_strength / self.scale) * self.wiggle_freq
        self.apply_force(thrust)

        self.velocity += self.acceleration
        if self.velocity.length() > self.max_speed:
            self.velocity.scale_to_length(self.max_speed)
        elif self.velocity.length() < self.min_speed:
            self.velocity.scale_to_length(self.min_speed)
        self.head.position += self.velocity
        self.acceleration *= 0

        # body physics
        for i in range(1, len(self.points)):
            p1 = self.points[i]
            p2 = self.points[i-1]
            
            dist = (p1.position - p2.position).length()
            if dist > p2.radius * 0.9:
                dir_vec = (p1.position - p2.position).normalize()
                p1.position = p2.position + dir_vec * p2.radius * 0.9
            
            if i < 2:
                continue
            
            p3 = self.points[i-2]
            anchor_angle = atan2(p2.position.y - p3.position.y, p2.position.x - p3.position.x)
            current_angle = atan2(p1.position.y - p2.position.y, p1.position.x - p2.position.x)
            
            angle_constraint = 0.5 + (0.3 * sin(self.wiggle_phase + i*0.2))
            new_angle = constrain_angle(current_angle, anchor_angle, angle_constraint)
            p1.position = p2.position + Vector2(cos(new_angle), sin(new_angle)) * (p2.radius * 0.9)

        self.wiggle_phase += 0.1 * self.wiggle_freq
        self.velocity *= 0.8 # damping
        self.update_scale(self.scale * (1 - (1/(FPS * 100) * 0.1))) # slightly reduce size over time (0.1% per second)
        
        # border repulsion (the fish government doesnt like emigrants)
        margin = self.head.radius * 5
        avoid_force = Vector2(0, 0)
        if self.head.position.x < margin:
            avoid_force.x = 2
        elif self.head.position.x > WIDTH - margin:
            avoid_force.x = -2
        if self.head.position.y < margin:
            avoid_force.y = 2
        elif self.head.position.y > HEIGHT - margin:
            avoid_force.y = -2
        if avoid_force.length() > 0:
            avoid_force.normalize_ip()
            self.apply_force(avoid_force * self.max_force * 5)
        
        # borders are FORGIVING !! fish will go back by themselves (fish government)
        if not (0 <= self.head.position.x <= WIDTH and 0 <= self.head.position.y <= HEIGHT) and not self.out_of_water:
            self.out_of_water = True
            self.last_out_time = clock.get_time()
        else:
            self.out_of_water = False
        
        # kill after 10 seconds (the fish had its chances)
        if clock.get_time() - self.last_out_time > 10 and self.out_of_water:
            self.update_scale(self.scale ** 0.9)
            self.respawn()

    def respawn(self):
        new_center = Vector2(uniform(0, WIDTH), uniform(0, HEIGHT))
        self.points = [Point(Vector2(new_center.x - i, new_center.y), self.radii[i]) for i in range(len(self.radii))]
        self.head = self.points[0]
        self.tail = self.points[-1]
        self.velocity = Vector2(0, 0)
        self.acceleration = Vector2(0, 0)
        self.wander_angle = random() * 2 * pi
        self.out_of_water = False

    def update_scale(self, new_scale: float):
        self.scale = max(self.min_scale, new_scale)
        if self.scale > self.max_scale:
            self.update_scale(self.scale / 3)
        self.thrust_strength = 0.25 * self.scale
        self.max_speed = 50 / self.scale
        self.radii = [self.scale * (-0.5 * (i-2.7)**2 + 20) if i < 8 else self.scale * (15 - i+2) for i in range(self.length)]
        for i in range(len(self.points)):
            if i < len(self.radii):
                self.points[i].radius = self.radii[i]
    
    def eat_food(self):
        self.update_scale(min(self.scale * 1.1, 3))


def constrain_angle(angle: float, anchor: float, constraint: float):
    diff = relative_angle_difference(angle, anchor)
    if abs(diff) <= constraint:
        return angle
    return anchor + constraint if diff > 0 else anchor - constraint

def relative_angle_difference(angle: float, anchor: float):
    return (angle - anchor + pi) % (2*pi) - pi

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

try:
    food_image = image.load("shrimp.png")
    food_image = transform.scale(food_image, (FOOD_RADIUS * 2, FOOD_RADIUS * 2))
except FileNotFoundError:
    food_image = None
if food_image:
    def draw_food():
        for food in food_list:
            screen.blit(food_image, rect.Rect(int(food.x), int(food.y), 1, 1))
else:
    def draw_food():
        for food in food_list:
            draw.circle(screen, (150, 100, 50), (int(food.x), int(food.y)), FOOD_RADIUS, FOOD_RADIUS//2)

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

    eye_radius = int(fish.scale * 5)
    for side in [-1, 1]:
        eye_pos = fish.head.position + (eyeline * (fish.scale * 10 * side))
        draw.circle(screen, (255, 255, 255), eye_pos, eye_radius)

def radix_sort(aquarium):
    max_val = max(fish.scale for fish in aquarium)
    exp = 1
    while max_val // exp > 0:
        count = [0] * 10
        output = [None] * len(aquarium)
        for fish in aquarium:
            index = int(fish.scale // exp)
            count[index % 10] += 1
        for i in range(1, 10):
            count[i] += count[i - 1]
        for i in range(len(aquarium) - 1, -1, -1):
            index = int(aquarium[i].scale // exp)
            output[count[index % 10] - 1] = aquarium[i]
            count[index % 10] -= 1
        for i in range(len(aquarium)):
            aquarium[i] = output[i]
        exp *= 10
    return aquarium


sin_timer = 0
wiggle_amplitude = 200
wiggle_frequency = 2 * pi

# ensure that the smallest fish is first !!
num_fish = 15
aquarium: list[Fish] = [Fish(scale=max(0.3, 5 / (i + 1.5))) for i in range(1, num_fish + 1)]
aquarium = radix_sort(aquarium)

def update_ecosystem(aquarium):
    predators = sorted(aquarium, key=lambda f: f.scale, reverse=True)[:2]
    for fish in aquarium:
        if fish not in predators:
            for food in food_list.copy():
                if fish.head.position.distance_to(food) < fish.head.radius + FOOD_RADIUS:
                    fish.eat_food()
                    food_list.remove(food)
    
    eaten = set()
    if aquarium:
        min_scale = min(f.scale for f in aquarium)
    else:
        min_scale = 0
    for fish in aquarium:
        if fish.scale == min_scale: # prey
            predators = [f for f in aquarium if f.scale > fish.scale]
            if predators:
                nearest_predator = min(predators, key=lambda p: p.head.position.distance_to(fish.head.position))
                if fish.head.position.distance_to(nearest_predator.head.position) < 400:
                    fish.flee(nearest_predator.head.position)
                    fish.seek(fish.head.position + Vector2(random() - 0.5, random() - 0.5) * 200)
                else:
                    if food_list:
                        closest_food = min(food_list, key=lambda food: fish.head.position.distance_to(food))
                        fish.seek(closest_food)
                    else:
                        if random() < 0.02:
                            fish.target = Vector2(randint(0, WIDTH), randint(0, HEIGHT))
                        fish.wander()
            else:
                if food_list:
                    closest_food = min(food_list, key=lambda food: fish.head.position.distance_to(food))
                    fish.seek(closest_food)
                else:
                    if random() < 0.02:
                        fish.target = Vector2(randint(0, WIDTH), randint(0, HEIGHT))
                    fish.wander()
        else: # predator
            candidates = [other for other in aquarium if other != fish and other.scale < fish.scale and other not in eaten]
            if candidates:
                prey = min(candidates, key=lambda p: fish.head.position.distance_to(p.head.position))
                dist = fish.head.position.distance_to(prey.head.position)
                if dist < 500:
                    predict_pos = prey.head.position + prey.velocity * 30 
                    fish.seek(predict_pos)
                    if dist < fish.head.radius:  # the bite of 87
                        fish.update_scale(fish.scale + prey.scale * 0.05)
                        prey.update_scale(prey.scale * 0.9)
                        prey.respawn()
                        fish.apply_force((predict_pos - fish.head.position).normalize() * 3)
                else:
                    fish.wander()
            else:
                fish.wander()
        fish.update()
    return aquarium

while True:
    for e in event.get():
        if e.type == QUIT:
            quit()
        elif e.type == VIDEORESIZE:
            WIDTH, HEIGHT = e.size
            screen = display.set_mode((WIDTH, HEIGHT), RESIZABLE)
    screen.fill((20, 20, 30))
    
    # (re)generate food
    current_time = time.get_ticks()
    if current_time - last_food_regen > 10000: # 10 seconds
        food_list = [Vector2(randint(0, WIDTH), randint(0, HEIGHT)) for _ in range(num_fish * 2)]
        last_food_regen = current_time

    draw_food()
    
    aquarium = update_ecosystem(aquarium)
    
    for fish in aquarium:
        draw_fish(fish)
    
    display.flip()
    clock.tick(FPS)