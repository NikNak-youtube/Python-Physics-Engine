import pygame
import physicsEngine
import threading

# Initialize pygame
pygame.init()

# Set up the display window
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Physics Simulation")

# Create a clock for controlling the frame rate
clock = pygame.time.Clock()
fps = 60


gridX = 100
gridY = 100

grid = [[0 for _ in range(gridX)] for _ in range(gridY)]



def verletSolveScreen():
    objects = physicsEngine.objects
    global grid
    
    # Fixed timestep for more stable simulation
    dt = 1.0 / fps
    
    # Store previous positions for interpolation
    for obj in objects:
        if not hasattr(obj, 'prev_position'):
            obj.prev_position = obj.position.copy()
        else:
            obj.prev_position = obj.position.copy()
    
    # Reset the grid to store object references
    grid = [[[] for _ in range(gridX)] for _ in range(gridY)]
    
    # Place objects in grid cells
    for i, obj in enumerate(objects):
        cell_x = min(99, max(0, int(obj.position[0] * 100 / width)))
        cell_y = min(99, max(0, int(obj.position[1] * 100 / height)))
        grid[cell_y][cell_x].append(i)
    
    # Single physics step per frame for more stable behavior
    # Update positions using Verlet integration
        
    for obj in objects:
        obj.update(dt)
        
        # Screen boundary collisions
        obj.screenCollisions(width, height)
        
        # Check for collisions
        collisions = obj.checkForCollision(grid, width, height)
        for collision in collisions:
            # Store pre-collision velocities and calculate total speed
            v1_before = [obj.velocity[0], obj.velocity[1]]
            v2_before = [collision.otherObject.velocity[0], collision.otherObject.velocity[1]]
            speed1_before = (v1_before[0]**2 + v1_before[1]**2)**0.5
            speed2_before = (v2_before[0]**2 + v2_before[1]**2)**0.5
            total_speed_before = speed1_before + speed2_before

            # Separate objects more gradually to reduce jitter
            position_correction_factor = 0.02  # Lower value for more gradual separation
            obj.velocity[0] -= collision.colNormal[0] * collision.overlap * position_correction_factor
            obj.velocity[1] -= collision.colNormal[1] * collision.overlap * position_correction_factor
            collision.otherObject.velocity[0] += collision.colNormal[0] * collision.overlap * position_correction_factor
            collision.otherObject.velocity[1] += collision.colNormal[1] * collision.overlap * position_correction_factor
            
            
            
            # Update velocities with reduced energy
            restitution = 1  # Lower restitution for less bouncy collisions

            # Calculate velocity along the collision normal
            v1_dot = obj.velocity[0] * collision.colNormal[0] + obj.velocity[1] * collision.colNormal[1]
            v2_dot = collision.otherObject.velocity[0] * collision.colNormal[0] + collision.otherObject.velocity[1] * collision.colNormal[1]
            
            # Only apply impulse if objects are moving toward each other
            relative_velocity = v1_dot - v2_dot
            if relative_velocity < 0:
                # Calculate impulse scalar with restitution
                impulse = -(1 + restitution) * relative_velocity
                impulse /= (1/obj.mass + 1/collision.otherObject.mass)
                
                # Apply impulse with dampening factor
                dampen = 0.5  # Lower dampening factor for smoother collisions
                obj.velocity[0] -= (impulse / obj.mass) * collision.colNormal[0] * dampen
                obj.velocity[1] -= (impulse / obj.mass) * collision.colNormal[1] * dampen
                collision.otherObject.velocity[0] += (impulse / collision.otherObject.mass) * collision.colNormal[0] * dampen
                collision.otherObject.velocity[1] += (impulse / collision.otherObject.mass) * collision.colNormal[1] * dampen
                
                '''
                # Calculate total speed after collision
                speed1_after = (obj.velocity[0]**2 + obj.velocity[1]**2)**0.5
                speed2_after = (collision.otherObject.velocity[0]**2 + collision.otherObject.velocity[1]**2)**0.5
                total_speed_after = speed1_after + speed2_after
                
                # If total speed increased, scale velocities down
                if total_speed_after > total_speed_before:
                    scale_factor = total_speed_before / total_speed_after
                    obj.velocity[0] *= scale_factor
                    obj.velocity[1] *= scale_factor
                    collision.otherObject.velocity[0] *= scale_factor
                    collision.otherObject.velocity[1] *= scale_factor'
                '''

airResistance = 0.00

# Create physics objects
physicsEngine.phyObject(20, 1, [100, 100], [0, 0], airResistance)

# Main loop
running = True
while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button
                mouse_pos = pygame.mouse.get_pos()
                physicsEngine.phyObject(20, 1, list(mouse_pos), [0,0], airResistance)
    
    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw objects
    for obj in physicsEngine.objects:
        obj.draw(screen)

    # Update physics
    verletSolveScreen()
    
    # Update the display
    pygame.display.flip()
    clock.tick(fps)