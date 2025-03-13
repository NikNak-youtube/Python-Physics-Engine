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



def verletSolveScreen(mostThreads = 4): 
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
    
    # Split objects into chunks for multithreading
    num_threads = min(mostThreads, len(objects))  # Use up to 4 threads, but no more than we have objects
    if num_threads <= 1:
        # Single-threaded path when few objects
        process_objects(objects, 0, len(objects), dt)
    else:
        # Multi-threaded path
        chunk_size = len(objects) // num_threads
        threads = []
        
        # Create and start threads
        for i in range(num_threads):
            start_idx = i * chunk_size
            end_idx = start_idx + chunk_size if i < num_threads - 1 else len(objects)
            thread = threading.Thread(
                target=process_objects,
                args=(objects, start_idx, end_idx, dt)
            )
            threads.append(thread)
            thread.start()
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
            
    # Handle collisions sequentially to ensure determinism
    resolve_all_collisions(objects)

def process_objects(objects, start_idx, end_idx, dt):
    """Process a subset of objects for multithreading"""
    for i in range(start_idx, end_idx):
        obj = objects[i]
        obj.update(dt)
        # Screen boundary collisions
        obj.screenCollisions(width, height)

def resolve_all_collisions(objects):
    """Resolve collisions deterministically"""
    # Detect all collisions first
    all_collisions = []
    for obj in objects:
        collisions = obj.checkForCollision(grid, width, height)
        for collision in collisions:
            # Add a unique identifier for each collision pair to avoid duplicates
            obj_id = id(obj)
            other_id = id(collision.otherObject)
            pair_id = tuple(sorted([obj_id, other_id]))
            all_collisions.append((pair_id, obj, collision))
    
    # Remove duplicates (same collision from both objects' perspectives)
    unique_collisions = {}
    for pair_id, obj, collision in all_collisions:
        if pair_id not in unique_collisions:
            unique_collisions[pair_id] = (obj, collision)
    
    # Sort collisions by pair_id for determinism
    sorted_collisions = sorted(unique_collisions.items())
    
    # Process collisions in a deterministic order
    for pair_id, (obj, collision) in sorted_collisions:
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
    verletSolveScreen(12)
    
    # Update the display
    pygame.display.flip()
    clock.tick(fps)