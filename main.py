"""
AI-Based Autonomous Navigation System - Main Application Entry Point.
Coordinates Perception, Path Planning, Kinematics, Control, and Visualization.
"""

import sys
import os
import math
import time
import argparse

# Add project root to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from src.config import (
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    FPS,
)
from src.simulation.environment import Environment
from src.simulation.vehicle import AutonomousVehicle
from src.simulation.sensors import LiDARSensor, CameraSensor
from src.perception.lidar_processor import LiDARProcessor
from src.planning.a_star import AStarPlanner
from src.planning.path_smoother import PathSmoother
from src.control.pid_controller import PIDController
from src.control.pure_pursuit import PurePursuitController

# Optional UI modules (pygame + cv2)
try:
    import pygame
    import cv2
    from src.perception.vision_detector import VisionPerception
    from src.ui.dashboard import Dashboard
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False


def plan_new_trajectory(environment, vehicle_pos, goal_pos):
    """
    Computes optimal collision-free path using A* and Thrun's path smoother.
    """
    start_cell = environment.world_to_grid(vehicle_pos[0], vehicle_pos[1])
    goal_cell = environment.world_to_grid(goal_pos[0], goal_pos[1])

    grid = environment.get_occupancy_grid(inflate_margin=1)
    planner = AStarPlanner(grid)
    grid_path = planner.plan(start_cell, goal_cell)

    if not grid_path:
        print("[NAVIGATION] Warning: No path found by A*. Obstacle blocking route.")
        return []

    # Smooth discrete grid path into continuous curve
    smoothed_path = PathSmoother.smooth(grid_path, environment)
    interpolated_path = PathSmoother.interpolate_waypoints(smoothed_path, step_size=12.0)
    return interpolated_path


def main():
    parser = argparse.ArgumentParser(description="AI-Based Autonomous Navigation System")
    parser.add_argument("--map", type=str, default="city", choices=["city", "warehouse"], help="Map environment")
    parser.add_argument("--backend", type=str, default="auto", choices=["auto", "pygame", "matplotlib"], help="UI rendering backend")
    parser.add_argument("--headless", action="store_true", help="Run without UI display for automated testing")
    parser.add_argument("--export-proof", action="store_true", help="Render and save proof snapshot to outputs/screenshots/")
    args = parser.parse_args()

    print("================================================================")
    print("      AI-BASED AUTONOMOUS NAVIGATION SYSTEM (AV-SIM 2D)         ")
    print("================================================================")
    print(f"[*] Map selected: {args.map.upper()}")
    print("[*] Initializing Kinematics, Perception, and Planning modules...")

    # Ensure output directories exist
    os.makedirs("outputs/screenshots", exist_ok=True)

    # 1. Initialize Subsystems
    env = Environment(map_name=args.map)
    vehicle = AutonomousVehicle(x=env.start_pos[0], y=env.start_pos[1], heading=0.0)
    lidar = LiDARSensor()
    camera = CameraSensor()
    lidar_processor = LiDARProcessor()
    pid = PIDController(kp=0.8, ki=0.04, kd=0.1)
    controller = PurePursuitController()

    # Plan initial path
    current_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
    print(f"[*] Initial A* Path Computed: {len(current_path)} continuous waypoints.")

    if args.headless:
        print("[*] Running in headless test mode (60 iterations)...")
        for _ in range(60):
            env.update()
            lidar_res = lidar.update((vehicle.x, vehicle.y), vehicle.heading, env.get_all_obstacles())
            summary = lidar_processor.process_scan(lidar_res["readings"])
            steer_cmd, target_speed, reached = controller.compute_steering(vehicle, current_path, summary)
            throttle = pid.compute(target_speed, vehicle.velocity)
            vehicle.apply_control(throttle, steer_cmd)
            vehicle.step_kinematics()
        print("[*] Headless test completed successfully! Vehicle reached distance:", round(vehicle.distance_traveled, 1))
        return

    # Check backend selection
    selected_backend = args.backend
    has_pygame = False
    try:
        import pygame
        import cv2
        has_pygame = True
    except ImportError:
        has_pygame = False

    if selected_backend == "auto":
        selected_backend = "pygame" if has_pygame else "matplotlib"

    if selected_backend == "matplotlib" or args.export_proof:
        from src.ui.matplotlib_dashboard import MatplotlibDashboard
        dash = MatplotlibDashboard(env, vehicle, plan_new_trajectory, controller, lidar, lidar_processor)
        if args.export_proof:
            print(f"[*] Simulating 80 frames on {args.map.upper()} map and capturing proof screenshot...")
            for i in range(80):
                dash.update_frame(i)
            proof_file = f"outputs/screenshots/simulation_proof_{args.map}.png"
            dash.fig.savefig(proof_file, dpi=120, bbox_inches="tight")
            print(f"[+] Verified proof snapshot saved to: {proof_file}")
            return
        dash.run(save_gif=False)
        return

    # Initialize Pygame Visual Dashboard & Vision Engine
    ui = Dashboard(width=SCREEN_WIDTH, height=SCREEN_HEIGHT)
    vision_engine = VisionPerception()

    running = True
    paused = False
    last_replan_time = time.time()
    screenshot_counter = 1

    print("[*] Simulation Started! Press [SPACE] to pause, [R] to reset, [S] for screenshot.")

    while running:
        fps_val = ui.clock.get_fps()

        # Handle UI events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                    print(f"[*] Simulation {'PAUSED' if paused else 'RESUMED'}")
                elif event.key == pygame.K_r:
                    # Reset vehicle and replan
                    vehicle.reset()
                    pid.reset()
                    controller.reset()
                    current_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
                    print("[*] Vehicle state reset to origin.")
                elif event.key == pygame.K_m:
                    # Switch map
                    new_map = "warehouse" if env.width and args.map == "city" else "city"
                    args.map = new_map
                    env.load_map(new_map)
                    vehicle.initial_pos = env.start_pos
                    vehicle.reset()
                    current_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
                    controller.reset()
                    print(f"[*] Switched to map: {new_map.upper()}")
                elif event.key == pygame.K_s:
                    # Capture screenshot
                    shot_path = f"outputs/screenshots/simulation_proof_{screenshot_counter}.png"
                    pygame.image.save(ui.screen, shot_path)
                    print(f"[+] Proof screenshot saved to: {shot_path}")
                    screenshot_counter += 1

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                if event.button == 1:
                    # Left Click: set new goal
                    env.goal_pos = (mx, my)
                    current_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
                    controller.reset()
                    print(f"[*] New destination waypoint set at: ({mx}, {my})")
                elif event.button == 3:
                    # Right Click: spawn obstacle block
                    obs_size = 40
                    env.static_obstacles.append((mx - obs_size // 2, my - obs_size // 2, obs_size, obs_size))
                    current_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
                    print(f"[!] Dynamic obstacle placed at: ({mx}, {my}). Re-routing...")

        if not paused:
            # 1. Update World Environment
            env.update()

            # 2. Check Collisions
            if env.check_collision(vehicle):
                vehicle.collision = True
                vehicle.state = "EMERGENCY_STOP"

            # 3. Perception: Virtual LiDAR Update
            all_obstacles = env.get_all_obstacles()
            lidar_data = lidar.update(
                (vehicle.x, vehicle.y),
                vehicle.heading,
                all_obstacles,
                bounds=(SCREEN_WIDTH, SCREEN_HEIGHT),
            )
            lidar_summary = lidar_processor.process_scan(lidar_data["readings"])

            # 4. Perception: Monocular Camera & Lane Detection
            camera_detections = camera.update((vehicle.x, vehicle.y), vehicle.heading, all_obstacles)
            synth_cam_feed = vision_engine.synthesize_camera_view(
                vehicle,
                camera_detections,
                steering_angle_deg=math.degrees(vehicle.steering_angle),
            )
            cv_annotated_feed, _ = vision_engine.process_lane_detection(synth_cam_feed)

            # 5. Planning & Control
            # Check dynamic re-planning trigger
            now = time.time()
            if controller.replan_needed and (now - last_replan_time > 1.5):
                print("[!] Local path blocked by obstacle. Re-planning global trajectory with A*...")
                new_path = plan_new_trajectory(env, (vehicle.x, vehicle.y), env.goal_pos)
                if new_path:
                    current_path = new_path
                    controller.reset()
                last_replan_time = now

            # Lateral steering control (Pure Pursuit)
            steer_cmd, target_speed, reached = controller.compute_steering(
                vehicle, current_path, lidar_summary
            )

            if reached:
                vehicle.state = "GOAL_REACHED"
                target_speed = 0.0

            # Longitudinal velocity control (PID)
            throttle_cmd = pid.compute(target_speed, vehicle.velocity)
            emergency = vehicle.collision or (lidar_summary["hazard_level"] == "CRITICAL" and target_speed < 0.1)

            vehicle.apply_control(throttle_cmd, steer_cmd, emergency_brake=emergency)
            vehicle.step_kinematics()

            # 6. Render combined simulation dashboard
            ui.render_frame(
                environment=env,
                vehicle=vehicle,
                path=current_path,
                lidar_data=lidar_data,
                lidar_summary=lidar_summary,
                cv_frame=cv_annotated_feed,
                target_speed=target_speed,
                fps_val=fps_val,
            )
        else:
            # Just maintain frame display while paused
            ui.clock.tick(30)

    pygame.quit()
    print("[*] Autonomous Navigation Simulation terminated cleanly.")


if __name__ == "__main__":
    main()
