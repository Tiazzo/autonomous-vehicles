import os

import sumolib
import traci

from models.base_model import BaseDecisionModel

# Settings
DELAY = "225"
BEGIN_TIME = "100"


def kmh_2_ms(speed: float) -> float:
    return speed / 3.6


class SimulationManager:
    def __init__(
        self, scenario: str, models: dict[str, BaseDecisionModel], max_steps: int = 500
    ):
        # Determine the correct path separator based on OS
        if os.name == "nt":
            self.config_file = os.path.join("scenarios", scenario, "simulation.sumocfg")
        else:
            self.config_file = f"scenarios/{scenario}/simulation.sumocfg"
        self.models = models  # {'ModelName': model_instance}
        self.max_steps = max_steps

    def get_sumo_cmd(self, sumo_binary: str = "sumo"):
        sumo_binary_path = sumolib.checkBinary(sumo_binary)
        return [
            sumo_binary_path,
            "-c",
            self.config_file,
            "--log",
            "sumo_log.txt",
            "--delay",
            DELAY,
            "-b",
            BEGIN_TIME,
        ]

    def run_simulation_for_model(
        self, model_name: str, model_instance: BaseDecisionModel
    ):
        traci.start(self.get_sumo_cmd(sumo_binary="sumo-gui"))
        # Disable default lane change logic for all vehicles
        for veh_id in traci.vehicle.getIDList():
            traci.vehicle.setLaneChangeMode(veh_id, 0)
            # traci.vehicle.setSpeedMode(veh_id, 0)

        step = 0
        while step < self.max_steps:
            traci.simulationStep()
            step += 1
            # Check if ego vehicle is in simulation:
            if "Ego" in traci.vehicle.getIDList():
                print("Ego vehicle is in simulation")

                veh_id = "Ego"
                current_lane = traci.vehicle.getLaneID(veh_id)

                # desired_lane_id = self.get_lane_id(edge, desired_lane_idx)
                # vehicle is already in the desired lane; no action needed
                print("current lane", current_lane)
                x = current_lane.split("_")
                if int(x[-1]) <= 0:
                    desired_lane = current_lane
                else:
                    desired_lane_idx = int(x[-1]) - 1
                    desired_lane = x[0] + "_" + str(desired_lane_idx)
                print("desired lane", desired_lane)
                if desired_lane == current_lane:
                    continue
                # prepare arguments for the decision model
                decision_kwargs = {
                    "veh_id": veh_id,
                    "current_lane": current_lane,
                    "desired_lane": desired_lane,
                }
                # invoke the decision model

                should_change_lane = model_instance.decide_lane_change(
                    **decision_kwargs
                )
                print("lane change: ", should_change_lane)

                if should_change_lane and not desired_lane == current_lane:
                    traci.vehicle.changeLane(veh_id, int(desired_lane_idx), 20)
                else:
                    # implement slowing down if safety metric is not met
                    # this requires fetching the safety metric, which the model does internally
                    # for simplicity, let's assume we slow down if a lane change was considered but not executed
                    current_speed = traci.vehicle.getSpeed(veh_id)
                    target_speed = max(
                        current_speed - kmh_2_ms(2), 0
                    )  # slow down by 2 km/h
                    duration = 5
                    traci.vehicle.slowDown(veh_id, target_speed, duration)

        traci.close()

    # def compute_desired_lanes(self):
    #     """
    #     Determines the desired lane for each vehicle based on its route.
    #     Returns a dictionary mapping vehicle IDs to desired lane indices.
    #     """
    #     desired_lanes = {}
    #     for veh_id in traci.vehicle.getIDList():
    #         route = traci.vehicle.getRoute(veh_id)
    #         current_edge = traci.vehicle.getRoadID(veh_id)
    #         if current_edge in route:
    #             current_edge_index = route.index(current_edge)
    #             if current_edge_index + 1 < len(route):
    #                 next_edge = route[current_edge_index + 1]
    #                 desired_lane = self.get_lane_for_edge(veh_id, next_edge)
    #                 desired_lanes[veh_id] = desired_lane
    #             else:
    #                 # Vehicle is on its last edge; retain current lane
    #                 edge, lane_idx = self._parse_lane_id(traci.vehicle.getLaneID(veh_id))
    #                 desired_lanes[veh_id] = lane_idx
    #         else:
    #             # Current edge not in route; retain current lane
    #             edge, lane_idx = self._parse_lane_id(traci.vehicle.getLaneID(veh_id))
    #             desired_lanes[veh_id] = lane_idx

    #     return desired_lanes

    # def get_lane_for_edge(self, veh_id: str, next_edge: str) -> str:
    #     """
    #     Maps the next edge to a desired lane index.
    #     This mapping should be defined based on the network topology and lane assignments.
    #     For simplicity, this example assumes:
    #         - If the next edge is E1, stay in lane 0
    #         - If the next edge is E2, switch to lane 1
    #     Adjust this mapping based on your specific network.
    #     """
    #     if next_edge == "E1":
    #         return "0"
    #     elif next_edge == "E2":
    #         return "1"
    #     else:
    #         # Default lane if no specific mapping is defined
    #         _, lane_idx = self._parse_lane_id(traci.vehicle.getLaneID(veh_id))
    #         return lane_idx

    # def get_lane_id(self, edge: str, lane_idx: str) -> str:
    #     """
    #     Constructs the lane ID based on edge and lane index.
    #     Assumes lane ID format is <edge>_<lane_index>.
    #     """
    #     return f"{edge}_{lane_idx}"

    # def _parse_lane_id(self, lane_id: str):
    #     """
    #     Parses lane ID to extract edge and lane index.
    #     Assumes lane ID format is <edge>_<lane_index>.
    #     Handles edge IDs that might contain underscores.
    #     """
    #     if "_" in lane_id:
    #         parts = lane_id.split("_")
    #         edge = "_".join(parts[:-1])  # Handle edge IDs with underscores
    #         lane_idx = parts[-1]
    #         return edge, lane_idx
    #     return None, None

    def run_all_simulations(self):
        print("run_all_simulations")
        for model_name, model_instance in self.models.items():
            print(f"Starting simulation for model: {model_name}")
            self.run_simulation_for_model(model_name, model_instance)
            print(f"Completed simulation for model: {model_name}")
