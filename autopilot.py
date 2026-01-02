#Launch a vehicle in CARLA with autopilot enabled and attach a camera to it
import carla
import random
import cv2
import numpy as np

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    vehicle_bp = random.choice(bp_lib.filter('vehicle.*'))
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    if vehicle is None:
        print("Failed to spawn vehicle")
        return

    vehicle.set_autopilot(True)

    # Spectator view
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    spectator.set_transform(
        carla.Transform(
            transform.location + carla.Location(x=-4, z=2.5),
            transform.rotation
        )
    )

    # Attach a camera to the vehicle
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Set up a callback to process images
    image_data = {"frame": None}

    def camera_callback(image):
        img = np.frombuffer(image.raw_data, dtype=np.uint8)
        img = img.reshape((image.height, image.width, 4))
        img = img[:, :, :3]        
        image_data["frame"] = img

    camera.listen(camera_callback)

    # Display the camera feed
    cv2.namedWindow("Camera Feed", cv2.WINDOW_AUTOSIZE)
    
    # Keep the script running to view the camera feed
    try:
        while True:
            if image_data["frame"] is not None:
                cv2.imshow("Camera Feed", image_data["frame"])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
