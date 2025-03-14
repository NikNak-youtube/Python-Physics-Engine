import pygame
import physicsEngine
import threading
import queue

# Initialize pygame
pygame.init()

# Set up the display window
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Physics Simulation")

# Create a clock for controlling the frame rate
clock = pygame.time.Clock()
fps = 60


gridX = int(800/40)
gridY = int(600/40)

grid = [[0 for _ in range(gridX)] for _ in range(gridY)]

# Thread Pool
class ThreadPool:
    def __init__(self, num_threads):
        self.num_threads = num_threads
        self.threads = []
        self.queue = queue.Queue()
        self.done = False
        
        # Create and start threads
        for _ in range(num_threads):
            thread = threading.Thread(target=self.worker)
            thread.start()
            self.threads.append(thread)
    
    def worker(self):
        while not self.done:
            try:
                task = self.queue.get(timeout=0.1)
                task()
            except queue.Empty:
                pass
    
    def add_task(self, task):
        self.queue.put(task)
    
    def wait_completion(self):
        self.queue.join()
        self.done = True
        for thread in self.threads:
            thread.join()


def verletSolveScreen(mostThreads = 4, dt = 1.0 / fps): 
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
        cell_x = min(gridX-1, max(0, int(obj.position[0] * gridX / width)))
        cell_y = min(gridY-1, max(0, int(obj.position[1] * gridY / height)))
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
        collisions = obj.checkForCollision(grid, width, height, gridX, gridY)
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
        # Check for more precise collision using circleCast from previous position
        other_obj = collision.otherObject
        
        # Use circleCast to check if objects would have collided along their paths
        hit_obj = physicsEngine.circleCast(
            obj.prev_position,
            obj.position,
            obj.radius*1000,
            grid,
            width,
            height,
            gridX,
            gridY
        )
        
        # If circleCast found a collision point, use it for more accurate resolution
        if hit_obj and hit_obj.id == other_obj.id:
            # Calculate collision normal more precisely
            dx = obj.position[0] - other_obj.position[0]
            dy = obj.position[1] - other_obj.position[1]
            distance = max(0.0001, (dx**2 + dy**2)**0.5)  # Avoid division by zero
            collision.colNormal = [dx/distance, dy/distance]
        
        # Store pre-collision velocities
        v1_before = [obj.velocity[0], obj.velocity[1]]
        v2_before = [other_obj.velocity[0], other_obj.velocity[1]]
        
        # Calculate velocity along the collision normal
        v1_dot = obj.velocity[0] * collision.colNormal[0] + obj.velocity[1] * collision.colNormal[1]
        v2_dot = other_obj.velocity[0] * collision.colNormal[0] + other_obj.velocity[1] * collision.colNormal[1]
        
        # Only apply impulse if objects are moving toward each other
        relative_velocity = v1_dot - v2_dot
        if relative_velocity < 0:
            # Separate objects to prevent sticking
            separation_factor = 0.02
            obj.position[0] += collision.colNormal[0] * collision.overlap * separation_factor
            obj.position[1] += collision.colNormal[1] * collision.overlap * separation_factor
            other_obj.position[0] -= collision.colNormal[0] * collision.overlap * separation_factor
            other_obj.position[1] -= collision.colNormal[1] * collision.overlap * separation_factor
            
            # Calculate impulse with restitution
            restitution = 0.8  # Coefficient of restitution (0.8 = slightly less bouncy)
            impulse = -(1 + restitution) * relative_velocity
            impulse /= (1/obj.mass + 1/other_obj.mass)
            
            # Apply impulse
            obj.velocity[0] += (impulse / obj.mass) * collision.colNormal[0]
            obj.velocity[1] += (impulse / obj.mass) * collision.colNormal[1]
            other_obj.velocity[0] -= (impulse / other_obj.mass) * collision.colNormal[0]
            other_obj.velocity[1] -= (impulse / other_obj.mass) * collision.colNormal[1]

            

airResistance = 0.00
radius = 5

# Create physics objects
physicsEngine.phyObject(radius, 1, [100, 100], [0, 0], airResistance)

# Main loop
running = True
while running:
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button
                # Store state for rapid fire
                pygame._mouse_held = True
                pygame._last_spawn_time = pygame.time.get_ticks()
                # Create initial object
                mouse_pos = pygame.mouse.get_pos()
                physicsEngine.phyObject(radius, 1, list(mouse_pos), [0,0], airResistance)
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # Left mouse button
                pygame._mouse_held = False
    
    # Initialize mouse state if not already set
    if not hasattr(pygame, '_mouse_held'):
        pygame._mouse_held = False
        pygame._last_spawn_time = 0
    
    # Rapid fire creation while mouse is held down
    if pygame._mouse_held:
        current_time = pygame.time.get_ticks()
        if current_time - pygame._last_spawn_time > 100:  # Create new object every 100ms
            mouse_pos = pygame.mouse.get_pos()
            physicsEngine.phyObject(radius, 1, list(mouse_pos), [0,0], airResistance)
            pygame._last_spawn_time = current_time
    
    # Clear the screen
    screen.fill((0, 0, 0))

    # Draw objects
    for obj in physicsEngine.objects:
        obj.draw(screen)

    # Update physics
    verletSolveScreen(1)
    
    # Update the display
    pygame.display.flip()
    clock.tick(fps)