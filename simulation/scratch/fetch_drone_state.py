import asyncio
from mavsdk import System
import time

async def get_drone_pose():
    # Create a drone object
    drone = System()

    # Connect to the drone
    await drone.connect(system_address="udp://:14553")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered")
            break

    # # Wait for the drone to have a global position estimate
    # print("Waiting for drone to have a global position estimate...")
    # async for health in drone.telemetry.health():
    #     if health.is_global_position_ok and health.is_home_position_ok:
    #         print("Global position estimate ok")
    #         break

    # Set the telemetry update rates
    await drone.telemetry.set_rate_position(60.0)  # Set position update rate to 60 Hz
    await drone.telemetry.set_rate_attitude_euler(60.0)  # Set attitude update rate to 60 Hz

    # Initialize variables to calculate message rate
    last_position_time = time.time()
    last_attitude_time = time.time()
    position_count = 0
    attitude_count = 0

    print("Fetching drone pose at 60Hz...")

    while True:
        # Get the drone's position
        async for position in drone.telemetry.position():
            position_count += 1
            current_time = time.time()
            elapsed_time = current_time - last_position_time
            if elapsed_time >= 1.0:
                rate = position_count / elapsed_time
                print(f"Position update rate: {rate:.2f} Hz")
                position_count = 0
                last_position_time = current_time
            print(f"Latitude: {position.latitude_deg}, Longitude: {position.longitude_deg}, Absolute Altitude: {position.absolute_altitude_m}")
            break

        # Get the drone's attitude
        async for attitude in drone.telemetry.attitude_euler():
            attitude_count += 1
            current_time = time.time()
            elapsed_time = current_time - last_attitude_time
            if elapsed_time >= 1.0:
                rate = attitude_count / elapsed_time
                print(f"Attitude update rate: {rate:.2f} Hz")
                attitude_count = 0
                last_attitude_time = current_time
            print(f"Roll: {attitude.roll_deg}, Pitch: {attitude.pitch_deg}, Yaw: {attitude.yaw_deg}")
            break
        # await drone.telemetry.set_rate_position(60.0)  # Set position update rate to 60 Hz
        # await drone.telemetry.set_rate_attitude_euler(60.0)  # Set attitude update rate to 60 Hz

if __name__ == "__main__":
    asyncio.run(get_drone_pose())
