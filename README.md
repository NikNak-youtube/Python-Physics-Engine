# Python Physics Engine

A 2D physics simulation engine built with Python and Pygame, featuring real-time collision detection, magnetic particle interactions, and multithreaded processing for performance optimization.

## Features

- **Real-time 2D Physics Simulation**: Verlet integration for stable physics calculations
- **Collision Detection**: Spatial grid-based collision detection with deterministic resolution
- **Magnetic Particles**: Electrostatic force simulation between charged particles
- **Multithreading**: Parallel processing of physics updates for improved performance
- **Interactive Controls**: Real-time parameter adjustment via sliders
- **Optimized Performance**: Grid-based spatial partitioning for efficient collision queries

## Installation

### Prerequisites

- Python 3.7 or higher
- pip package manager

### Setup

1. Clone or download this repository:

```bash
git clone https://github.com/NikNak-youtube/Python-Physics-Engine.git
cd Python-Physics-Engine
```

2. Install required dependencies:

```bash
pip install pygame
```

3. Run the simulation:

```bash
python physics.py
```

## How to Use

### Basic Controls

- **Left Click**: Spawn regular physics objects
- **Right Click**: Spawn positively charged magnetic particles (red)
- **Middle Click**: Spawn negatively charged magnetic particles (blue)
- **Hold Mouse**: Rapid-fire object creation
- **Spacebar**: Toggle parameter sliders panel

### Parameter Controls

When the slider panel is visible (press Spacebar), you can adjust:

- **Air Resistance**: Controls how quickly objects slow down (0.0 - 0.1)
- **Radius**: Size of newly created objects (5 - 30 pixels)
- **Mass**: Mass of newly created objects (1 - 10 units)

### Simulation Behavior

- Objects fall under gravity and bounce off screen boundaries
- Magnetic particles attract/repel based on charge (opposite charges attract, same charges repel)
- All objects experience air resistance and collision physics
- Collision resolution prevents objects from sticking together

## Programming with the Library

### Basic Usage

```python
import pygame
import physicsEngine

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((800, 600))

# Create physics objects
ball = physicsEngine.phyObject(
    radius=10,           # Object radius in pixels
    mass=1,             # Object mass
    position=[100, 100], # Starting position [x, y]
    velocity=[50, 0],   # Initial velocity [vx, vy]
    airResistance=0.03  # Air resistance coefficient
)

# Create magnetic particle
magnet = physicsEngine.magneticParticle(
    radius=15,
    mass=2,
    position=[200, 100],
    velocity=[0, 0],
    charge=1.0,         # Positive charge
    airResistance=0.03
)
```

### Custom Object Classes

Extend the base `phyObject` class to create custom physics objects:

```python
class CustomObject(physicsEngine.phyObject):
    def __init__(self, radius, mass, position, velocity):
        super().__init__(radius, mass, position, velocity)
        self.custom_property = "value"
    
    def applyForces(self):
        # Override to add custom forces
        acceleration = [0, 0]
        
        # Add gravity
        acceleration[1] += 9.8 * 20
        
        # Add custom force (e.g., wind)
        acceleration[0] += 5
        
        return acceleration
    
    def draw(self, screen):
        # Custom drawing
        pygame.draw.circle(screen, (0, 255, 0), 
                         (int(self.position[0]), int(self.position[1])), 
                         self.radius)
```

### Physics Loop Integration

```python
import physics  # Import the main physics module

# Set up grid for collision detection
gridX = int(800/40)  # Grid cells in X direction
gridY = int(600/40)  # Grid cells in Y direction
grid = [[[] for _ in range(gridX)] for _ in range(gridY)]

# Main game loop
while running:
    # Handle events
    for event in pygame.event.get():
        # ... event handling
    
    # Update physics
    physics.verletSolveScreen(mostThreads=4, dt=1.0/60)
    
    # Draw objects
    screen.fill((0, 0, 0))
    for obj in physicsEngine.objects:
        obj.draw(screen)
    
    pygame.display.flip()
```

## Technical Architecture

### Physics Integration

The engine uses **Verlet Integration** for numerical stability:

```text
position(t+dt) = position(t) + velocity(t)*dt + acceleration(t)*dt²/2
velocity(t+dt) = velocity(t) + (acceleration(t) + acceleration(t+dt))*dt/2
```

This method provides better stability than Euler integration, especially for collision-heavy simulations.

### Collision Detection System

**Spatial Grid Partitioning**:

- Screen space is divided into a uniform grid (default: 20x15 cells)
- Objects are assigned to grid cells based on position
- Collision checks only occur between objects in neighboring cells
- Reduces collision complexity from O(n²) to approximately O(n)

**Collision Resolution**:

1. **Detection**: Grid-based broad phase, distance-based narrow phase
2. **Separation**: Position correction to prevent object overlap
3. **Response**: Impulse-based velocity changes with restitution
4. **Determinism**: Collisions sorted by object ID pairs for consistent results

### Multithreading Architecture

**Thread Distribution**:

- Objects divided into chunks across available CPU cores
- Each thread processes physics updates independently
- Collision detection performed sequentially to maintain determinism
- Thread pool pattern minimizes thread creation overhead

**Synchronization**:

- Position updates parallelized safely (no shared state)
- Collision resolution serialized to prevent race conditions
- Grid updates synchronized between frames

### Magnetic Force Simulation

Implements simplified electrostatic force calculation:

```text
F = k * q1 * q2 / r²
```

Where:

- `k` is the electromagnetic constant (scaled for simulation)
- `q1`, `q2` are particle charges
- `r` is the distance between particles

Forces are applied as accelerations: `a = F/m`

## Abstract Design Principles

### Modular Architecture

- **physicsEngine.py**: Core physics classes and utilities
- **physics.py**: Main simulation loop and rendering
- **Separation of Concerns**: Physics logic separate from rendering

### Object-Oriented Design

- **Inheritance**: `magneticParticle` extends `phyObject`
- **Polymorphism**: Different object types use same interface
- **Encapsulation**: Physics state contained within objects

### Performance Optimization

- **Spatial Partitioning**: Reduces computational complexity
- **Parallel Processing**: Utilizes multiple CPU cores
- **Fixed Timestep**: Ensures simulation stability
- **Memory Efficiency**: Grid reused each frame, minimal allocations

### Deterministic Simulation

- **Reproducible Results**: Same inputs always produce same outputs
- **Sorted Processing**: Objects processed in consistent order
- **Thread-Safe Design**: Parallel regions carefully isolated

## Contributing

### Getting Started

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/your-feature-name`
3. Make your changes with clear, documented code
4. Test thoroughly with various scenarios
5. Submit a pull request with detailed description

### Code Style Guidelines

- Follow PEP 8 Python style conventions
- Use descriptive variable and function names
- Add docstrings for all public methods
- Include type hints where beneficial
- Comment complex algorithms and physics calculations

### Areas for Contribution

**Performance Improvements**:

- Optimize grid cell traversal algorithms
- Implement quadtree or octree spatial structures
- Add GPU acceleration with compute shaders
- Profile and optimize hot paths

**Physics Features**:

- Add spring/constraint systems
- Implement fluid dynamics
- Add rotational physics and torque
- Support for different collision shapes (polygons, etc.)

**Simulation Features**:

- Save/load simulation states
- Recording and playback functionality
- Real-time physics parameter graphing
- Multiple simulation scenes

**Code Quality**:

- Add comprehensive unit tests
- Implement continuous integration
- Add performance benchmarking
- Improve error handling and validation

### Testing

When contributing, ensure your changes:

- Don't break existing functionality
- Maintain physics simulation stability
- Preserve deterministic behavior
- Include appropriate test cases

### Reporting Issues

When reporting bugs or requesting features:

1. Check existing issues first
2. Provide clear reproduction steps
3. Include system information (Python version, OS)
4. Attach relevant code samples or screenshots
5. Describe expected vs actual behavior

## Performance Notes

- **Optimal Thread Count**: Usually matches CPU core count (4-8 threads)
- **Object Limits**: Efficiently handles 100-1000+ objects depending on hardware
- **Grid Resolution**: 40x40 pixel cells provide good performance/accuracy balance
- **Frame Rate**: Targets 60 FPS with automatic timestep adjustment

## License

This project is open source and available under the MIT License.

## Acknowledgments

- Built with [Pygame](https://www.pygame.org/) for graphics and input handling
- Inspired by real-time physics simulation techniques
- Physics algorithms based on established game development practices
