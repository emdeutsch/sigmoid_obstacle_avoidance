# Path follower using a simple bicycle model
# Multiple vehicles are used. The vehicles
# that aren't tracked are considered "opponents"
# Meant to demonstrate the WA Simulator API
# -----------------------------------------

# Import the simulator
import wa_simulator as wa

# Import the controller
from pid_controller import PIDController

# Other imports
import numpy as np

class SigmoidPIDController(PIDController):
    def __init__(self, system: wa.WASystem, vehicle: wa.WAVehicle, vehicle_inputs: wa.WAVehicleInputs, path: wa.WAPath, opponents: [wa.WAVehicle], lat_controller: 'PIDLateralController' = None, long_controller: 'PIDLongitudinalController' = None):
        super().__init__(system, vehicle, vehicle_inputs, path, lat_controller, long_controller)


def main():
    # ---------------
    # Create a system
    # System's handle the simulation settings
    system = wa.WASystem(step_size=1e-3)

    # ---------------------------
    # Create a simple environment
    # Environment will create a track like path for the vehicle
    env_filename = wa.WASimpleEnvironment.EGP_ENV_MODEL_FILE
    environment = wa.WASimpleEnvironment(system, env_filename)

    # --------------------------------
    # Create the vehicle inputs object
    # This is a shared object between controllers, visualizations and vehicles
    vehicle_inputs = wa.WAVehicleInputs()

    # ----------------
    # Create a vehicle
    # Uses a premade json specification file that describes the properties of the vehicle
    veh_filename = wa.WALinearKinematicBicycle.GO_KART_MODEL_FILE
    vehicle = wa.WALinearKinematicBicycle(system, vehicle_inputs, veh_filename)

    # Create Track
    track_data = read_file('./sigmoid-obstacle-avoidance/example_track.csv')
    left_points = []
    right_points = []
    for _ in track_data:
        if track_data['is_left'][0] == 0:
            left_points.append([track_data['x'][0], track_data['y'][0], 0])
        else:
            right_points.append([track_data['x'][0], track_data['y'][0], 0])
        track_data = np.delete(track_data, 0, axis=0)
    left = wa.WASplinePath(left_points, num_points=1000)
    right = wa.WASplinePath(right_points, num_points=1000)
    center_points = []
    for i in range(len(left_points)):
        leftPt = [left._x[i], left._y[i]]
        rightPt = [right._x[i], right._y[i]]

        centerX = (leftPt[0] + rightPt[0]) / 2
        centerY = (leftPt[1] + rightPt[1]) / 2
        center_points.append([centerX, centerY, 0])
    center = wa.WASplinePath(center_points, num_points=1000)
    track = wa.WATrack(center, left, right)

    # ------------------
    # Create n opponents
    opponents = []
    opponent_vehicle_inputs_list = []
    num_opponents = 1
    for i in range(num_opponents):
        opponent_vehicle_inputs = wa.WAVehicleInputs()
        opponent = wa.WALinearKinematicBicycle(system, vehicle_inputs, veh_filename)
        opponents.append(opponent)
        opponent_vehicle_inputs_list.append(opponent_vehicle_inputs)

    # ----------------------
    # Create a visualization
    # Will use matplotlib for visualization
    visualization = wa.WAMatplotlibVisualization(system, vehicle, vehicle_inputs, environment=environment, opponents=opponents)

    # -------------------
    # Create a controller
    # Create a pid controller
    controllers = [SigmoidPIDController(system, vehicle, vehicle_inputs, center, opponents)]
    for i in range(num_opponents):
        opponent_controller = PIDController(system, opponents[i], opponent_vehicle_inputs_list[i], center)
        controllers.append(opponent_controller)

    # --------------------------
    # Create a simuation wrapper
    # Will be responsible for actually running the simulation
    sim_manager = wa.WASimulationManager(system, vehicle, visualization, *controllers, *opponents)

    # ---------------
    # Simulation loop
    step_size = system.step_size
    while sim_manager.is_ok():
        time = system.time

        sim_manager.synchronize(time)
        sim_manager.advance(step_size)

def read_file(file):
    # a delimiter is the thing that separates each data value in a row
    # data is now a numpy array with our data
    data = np.genfromtxt(file, delimiter=',', names=True)

    # Errors may occur with the above method, like if delimiter is wrong or inconsistent names

    # Check to make sure the data is as we expected
    if data.dtype.names != ('is_left', 'x', 'y') or np.isnan([r.tolist() for r in data]).any():
        raise ValueError('The csv file is not structured incorrectly!')

    return data


if __name__ == "__main__":
    main()