# Launch a car in CARLA and position the camera behind it
import carla
import random
import time

# Connect to the CARLA server
client = carla.Client('localhost', 2000) 
client.set_timeout(10.0)   

# Get the world and the blueprint library
world = client.get_world()
bp_lib = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()
vehicle_bp = random.choice(bp_lib.filter('vehicle.*'))
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# Get the spectator camera and position it behind the vehicle
spectator = world.get_spectator()   
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x = -4, z = 2.5)), vehicle.get_transform().rotation)
spectator.set_transform(transform)

# Set AutoPilot
vehicle.set_autopilot(True)

# Let the vehicle drive for a while
time.sleep(10)

# Destroy the vehicle
vehicle.destroy()