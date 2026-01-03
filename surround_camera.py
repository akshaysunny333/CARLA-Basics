#Launch a CARLA simulation with a vehicle equipped with four surround cameras capturing only other vehicles in the scene.
import carla
import random
import cv2
import numpy as np

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    original_settings = world.get_settings()

    #Setting synchronous mode to reduce lag
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # Setting synchronous mode for traffic manager
    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)

    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    #Spawning a random 4-wheeled vehicle
    vehicle_bps = [
        bp for bp in bp_lib.filter('vehicle.*')
        if int(bp.get_attribute('number_of_wheels')) == 4
    ]

    vehicle_bp = random.choice(vehicle_bps)
    vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

    if vehicle is None:
        print("Failed to spawn vehicle")
        world.apply_settings(original_settings)
        return

    vehicle.set_autopilot(True, tm.get_port())

    #Setting up the surround cameras
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '480')
    camera_bp.set_attribute('image_size_y', '360')
    camera_bp.set_attribute('fov', '90')
    camera_bp.set_attribute('sensor_tick', '0.05')

    # Calculate camera positions based on vehicle bounding box
    bbox = vehicle.bounding_box
    cam_z = bbox.extent.z + 0.5

    camera_configs = {
        "front": carla.Transform(
            carla.Location(x=bbox.extent.x + 0.3, z=cam_z),
            carla.Rotation(yaw=0)
        ),
        "right": carla.Transform(
            carla.Location(y=bbox.extent.y + 0.3, z=cam_z),
            carla.Rotation(yaw=90)
        ),
        "rear": carla.Transform(
            carla.Location(x=-(bbox.extent.x + 0.3), z=cam_z),
            carla.Rotation(yaw=180)
        ),
        "left": carla.Transform(
            carla.Location(y=-(bbox.extent.y + 0.3), z=cam_z),
            carla.Rotation(yaw=-90)
        ),
    }

    cameras = {}
    images = {name: None for name in camera_configs.keys()}

    def make_callback(name):
        def callback(image):
            img = np.frombuffer(image.raw_data, dtype=np.uint8)
            img = img.reshape((image.height, image.width, 4))
            images[name] = img[:, :, :3]
        return callback

    for name, transform in camera_configs.items():
        cam = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        cam.listen(make_callback(name))
        cameras[name] = cam

    cv2.namedWindow("Surround View (Cars Only)", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Surround View (Cars Only)", 960, 720)

    try:
        while True:
            world.tick()

            if all(img is not None for img in images.values()):
                top = np.hstack((images["front"], images["right"]))
                bottom = np.hstack((images["left"], images["rear"]))
                view = np.vstack((top, bottom))
                cv2.imshow("Surround View (Cars Only)", view)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        for cam in cameras.values():
            cam.stop()
            cam.destroy()

        vehicle.destroy()

        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
