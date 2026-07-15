"""AT Praktikum Summer Semester 2026 - Controller Files"""

# Webots internal controller - not to be confused with the PID controllers below
from controller import Robot # type: ignore - run from Webots

# import sensor readings
from utils.sensors import Sensors

# import data logging and printing
from utils.logger import Logger

# import the flight controller factory
from controllers.controller_factory import ControllerType, create_flight_controller

from utils.motor_setup import initialize_motors, set_motor_velocities

# import trajectory_planner
from trajectory_planner.setpoint import Setpoint
from trajectory_planner.trajectory import Trajectory
from trajectory_planner.trajectory_tracker import TrajectoryTracker

# Main simulation loop - runs in loop until the user stops the simulation
def main():

    # create the Robot instance.
    robot = Robot()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    dt = timestep / 1000.0  # convert ms → seconds

    # Get sensors
    sensors = Sensors(robot, timestep)

    # One line to select and create the controller
    flight_controller = create_flight_controller(ControllerType.CASCADED, dt)

    # Get the logger
    logger = Logger(print_interval=1.0)

    # Diagnostics toggle
    DEBUG = False  # Set to False to silence all diagnostic prints

    # Initialize motors
    motors = initialize_motors(robot)

    # Experiment mode selection
    MODE = "STEP_RESPONSE"   # Options: "TRAJECTORY" or "STEP_RESPONSE"

    ## Trajectory Tracking
    
    # Reference Trajectory - array of setpoints
    trajectory = Trajectory([
        Setpoint(x=0.0, y=0.0, z=0.6, yaw= 0),
        Setpoint(x=-0.4, y=0.5, z=0.9, yaw= 0),
        Setpoint(x=-0.6, y=2.5, z=1.1, yaw= 0),
        Setpoint(x=-0.5, y=3.4, z=0.6, yaw= 0),
        Setpoint(x=-0.2, y=4.4, z=0.5, yaw= 0),
        Setpoint(x=0.3, y=5.4, z=0.4, yaw= 0),
        Setpoint(x=0.7, y=7.0, z=0.4, yaw= 0),
        Setpoint(x=0.4, y=7.5, z=0.4, yaw= 0),
        Setpoint(x=0, y=8.0, z=0.3, yaw= 0),
        Setpoint(x=0, y=8.0, z=0, yaw= 0)
    ])

    tracker = TrajectoryTracker(trajectory)
        
    ###########

    # Main loop:
    # - Perform simulation steps until Webots is stopped
    while robot.step(timestep) != -1:
    
        sim_time = robot.getTime()  # returns time in seconds
    
        # Read the sensors:
        state = sensors.read()

        # Step response finished flag
        step_response_finished = False
        
        if MODE == "TRAJECTORY":
            # Trajectory Tracker provides the setpoints for the controller to follow
            reference = tracker.update(state, sim_time)

        else:
            reference = Setpoint(
                    x=0.0,
                    y=0.0,
                    z=1.0, # Step response height of 1 meter
                    yaw=0.0
                ).to_dict()
            

        # Store step-response data for standardized 30 s plot
        if MODE == "STEP_RESPONSE":
            logger.record_step_response(state, reference, sim_time, max_log_time=30.0)


        # Flight Controller - CascadedController
        motor_velocities = flight_controller.compute_motor_commands(state, reference, debug=DEBUG)

        # Stop the motors once landed / experiment finished
        if MODE == "TRAJECTORY":

            if tracker.trajectory_completed and state["position"][2] < 0.05:
                motor_velocities = [0.0, 0.0, 0.0, 0.0]

        elif MODE == "STEP_RESPONSE":

            if sim_time > 30.1:
                motor_velocities = [0.0, 0.0, 0.0, 0.0]
                step_response_finished = True

        # Set motor velocities for each propeller
        set_motor_velocities(motors, motor_velocities)

        # Finish standardized step-response experiment
        if MODE == "STEP_RESPONSE" and step_response_finished:
            logger.finalize_step_response(
                filename="step_response_30s.png",
                target=1.0,
                tolerance=0.05
            )
            break

        # End of main loop

if __name__ == "__main__":
    main()