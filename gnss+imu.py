#Launch a CARLA simulation with a vehicle equipped with GNSS and IMU sensors.
import carla
import random
import cv2
import numpy as np

def main():
    # Connect to CARLA server and set up the world
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

    spectator = world.get_spectator()

    #Attaching GNSS and IMU sensors to the vehicle
    gnss_bp = bp_lib.find('sensor.other.gnss')
    gnss_transform = carla.Transform(carla.Location(x=0, z=2))
    gnss_sensor = world.spawn_actor(
        gnss_bp, gnss_transform, attach_to=vehicle
    )

    imu_bp = bp_lib.find('sensor.other.imu')
    imu_transform = carla.Transform(carla.Location(x=0, z=2))
    imu_sensor = world.spawn_actor(
        imu_bp, imu_transform, attach_to=vehicle
    )

    # Initialize sensor data containers
    gnss_data = {"lat": 0.0, "lon": 0.0, "alt": 0.0}
    imu_data = {
        "accel": (0.0, 0.0, 0.0),
        "gyro": (0.0, 0.0, 0.0),
        "compass": 0.0
    }

    # Sensor callback functions
    def gnss_callback(data):
        gnss_data["lat"] = data.latitude
        gnss_data["lon"] = data.longitude
        gnss_data["alt"] = data.altitude

    def imu_callback(data):
        imu_data["accel"] = (
            data.accelerometer.x,
            data.accelerometer.y,
            data.accelerometer.z
        )
        imu_data["gyro"] = (
            data.gyroscope.x,
            data.gyroscope.y,
            data.gyroscope.z
        )
        imu_data["compass"] = data.compass

    gnss_sensor.listen(gnss_callback)
    imu_sensor.listen(imu_callback)

    # Display GNSS and IMU data in a window
    cv2.namedWindow("GNSS & IMU Live Data", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # Spectator follows the vehicle
            vehicle_transform = vehicle.get_transform()

            spectator_transform = carla.Transform(
                vehicle_transform.location +
                vehicle_transform.get_forward_vector() * -8 +
                carla.Location(z=3),
                vehicle_transform.rotation
            )

            spectator_transform.rotation.pitch = -10
            spectator.set_transform(spectator_transform)

            # Create a dashboard to display GNSS and IMU data
            dashboard = np.zeros((360, 680, 3), dtype=np.uint8)

            cv2.putText(dashboard, "GNSS DATA", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            cv2.putText(dashboard, f"Latitude  : {gnss_data['lat']:.6f}",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)

            cv2.putText(dashboard, f"Longitude : {gnss_data['lon']:.6f}",
                        (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)

            cv2.putText(dashboard, f"Altitude  : {gnss_data['alt']:.2f} m",
                        (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 1)

            cv2.putText(dashboard, "IMU DATA", (20, 190),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            ax, ay, az = imu_data["accel"]
            gx, gy, gz = imu_data["gyro"]

            cv2.putText(
                dashboard,
                f"Accel (m/s²): x={ax:.2f}, y={ay:.2f}, z={az:.2f}",
                (20, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1
            )

            cv2.putText(
                dashboard,
                f"Gyro  (rad/s): x={gx:.2f}, y={gy:.2f}, z={gz:.2f}",
                (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1
            )

            cv2.putText(
                dashboard,
                f"Compass: {imu_data['compass']:.3f} rad",
                (20, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1
            )

            cv2.imshow("GNSS & IMU Live Data", dashboard)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        print("Cleaning up actors...")
        gnss_sensor.stop()
        imu_sensor.stop()
        gnss_sensor.destroy()
        imu_sensor.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
