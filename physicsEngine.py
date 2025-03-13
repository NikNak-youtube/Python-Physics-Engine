import pygame

objects = []

class phyCollision:
    colPoint = [0, 0]
    colNormal = [0, 0]
    colDepth = 0
    overlap = 0
    otherObject = None

    def __init__(self, colPoint, colNormal, colDepth, overlap, otherObject):
        self.colPoint = colPoint
        self.colNormal = colNormal
        self.colDepth = colDepth
        self.overlap = overlap
        self.otherObject = otherObject

class phyObject:
    radius = 1
    mass = 1
    position = [float(0), float(0)]
    velocity = [float(0), float(0)]
    acceleration = [float(0), float(0)]
    airResistance = 0.03

    def __init__(self, radius, mass, position, velocity, airResistance=0.03):
        self.radius = radius
        self.mass = mass
        self.position = position
        self.velocity = velocity
        self.airResistance = airResistance
        self.id = len(objects)
        self.showed_velocity = 5
        objects.append(self)

    def draw(self, screen):
        pygame.draw.circle(screen, (255, 255, 255), (int(self.position[0]), int(self.position[1])), self.radius)

    def applyForces(self):
        NewAcceleration = [0, 0]
        # Apply gravity
        NewAcceleration[1] += 9.8 * 20
        return NewAcceleration

    def update(self, dt):
        self.position[0] = self.position[0] + self.velocity[0] * dt + self.acceleration[0] * (dt**2 * 0.5)
        self.position[1] = self.position[1] + self.velocity[1] * dt + self.acceleration[1] * (dt**2 * 0.5)
        newAcceleration = self.applyForces()
        self.velocity[0] = self.velocity[0] + (self.acceleration[0] + newAcceleration[0]) * dt * 0.5
        self.velocity[1] = self.velocity[1] + (self.acceleration[1] + newAcceleration[1]) * dt * 0.5
        self.acceleration = newAcceleration
        self.velocity[0] = self.velocity[0] * (1 - self.airResistance)
        self.velocity[1] = self.velocity[1] * (1 - self.airResistance)
        if self.id == len(objects) - 1 and self.showed_velocity > 0:
            self.showed_velocity -= 1
            print(f"{self.id} {self.velocity} {self.acceleration} {newAcceleration} {dt}")
    
    # Add interpolated drawing method to phyObject
    def draw_interpolated(obj, alpha, screen):
        interp_x = obj.prev_position[0] * (1 - alpha) + obj.position[0] * alpha
        interp_y = obj.prev_position[1] * (1 - alpha) + obj.position[1] * alpha
        pygame.draw.circle(screen, (255, 255, 255), (int(interp_x), int(interp_y)), obj.radius)

    def screenCollisions(self, width, height):
        if self.position[0] < self.radius:
            self.position[0] = self.radius
            self.velocity[0] *= -0.9
        elif self.position[0] > width - self.radius:
            self.position[0] = width - self.radius
            self.velocity[0] *= -0.9
        
        if self.position[1] < self.radius:
            self.position[1] = self.radius
            self.velocity[1] *= -0.9
        elif self.position[1] > height - self.radius:
            self.position[1] = height - self.radius
            self.velocity[1] *= -0.9

    def checkForCollision(self, gridMap, width, height):
        collisions = []
        cell_x = min(99, max(0, int(self.position[0] * 100 / width)))
        cell_y = min(99, max(0, int(self.position[1] * 100 / height)))
        for y in range(3):
            for x in range(3):
                ny = cell_y + y - 1
                nx = cell_x + x - 1
                if 0 <= ny < 100 and 0 <= nx < 100 and gridMap[ny][nx]:
                    for obj_index in gridMap[ny][nx]:
                        obj = objects[obj_index]
                        if obj != self:
                            dx = self.position[0] - obj.position[0]
                            dy = self.position[1] - obj.position[1]
                            distance = (dx**2 + dy**2)**0.5
                            if distance < self.radius + obj.radius:
                                overlap = self.radius + obj.radius - distance
                                if distance > 0:
                                    dx /= distance
                                    dy /= distance
                                else:
                                    # If objects are exactly at the same position, use a default direction
                                    dx, dy = 1, 0
                                collision_x = (self.position[0] + obj.position[0]) / 2
                                collision_y = (self.position[1] + obj.position[1]) / 2
                                collisions.append(phyCollision([collision_x, collision_y], [dx, dy], overlap, overlap, obj))
        return collisions