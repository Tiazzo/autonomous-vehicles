import traci

from models.base_model import BaseDecisionModel

# SL2015 model parameters
STRATEGIC_PARAM = 1.0
SPEEDGAIN_PARAM = 1.0
KEEPRIGHT_PARAM = 1.0
MIN_SAFE_GAP = 15  # meters
REACTION_TIME = 2.0  # seconds

# Additional SL2015 parameters
LC_PARAMS = {
    "lcSublane": 0.5,
    "lcPushy": 0.1,
    "lcAssertive": 0.2,
    "lcImpatience": 0.0,
    "lcTimeToImpatience": 120.0,
    "lcAccelLat": 0.5,
    "maxSpeedLat": 0.5,
    "minGapLat": 1.5,
    "latAlignment": "center",
}


def kmh_2_ms(speed: float) -> float:
    return speed / 3.6


def distance_between(vehicle_1, vehicle_2):
    pos1 = traci.vehicle.getPosition(vehicle_1)
    pos2 = traci.vehicle.getPosition(vehicle_2)
    return pos2[0] - pos1[0]


def findclosest(vehicles, ego):
    if vehicles:
        closest_front = None
        closest_back = None
        min_front_distance = float("inf")
        min_back_distance = float("-inf")

        for veh in vehicles:
            dis = distance_between(veh, ego)
            if dis > 0 and dis < min_front_distance:
                min_front_distance = dis
                closest_front = veh
            if dis < 0 and dis > min_back_distance:
                min_back_distance = dis
                closest_back = veh
    else:
        closest_front = None
        closest_back = None
    return closest_front, closest_back


class SL2015(BaseDecisionModel):
    def __init__(self):
        super().__init__()
        self.lc_params = LC_PARAMS

    def set_vehicle_parameters(self, veh_id: str):
        """Set the lane-change parameters for the vehicle"""
        for param, value in self.lc_params.items():
            traci.vehicle.setParameter(veh_id, param, str(value))

    def decide_lane_change(
        self, veh_id: str, current_lane: str, desired_lane: str
    ) -> bool:
        # Set vehicle parameters if not already set
        self.set_vehicle_parameters(veh_id)

    def decide_lane_change(
        self, veh_id: str, current_lane: str, desired_lane: str
    ) -> bool:
        # Get vehicle states
        ego_speed = traci.vehicle.getSpeed(veh_id)
        ego_max_speed = traci.vehicle.getMaxSpeed(veh_id)

        # Get surrounding vehicles
        target_vehicles = traci.lane.getLastStepVehicleIDs(desired_lane)
        current_vehicles = traci.lane.getLastStepVehicleIDs(current_lane)

        leader_target, follower_target = findclosest(target_vehicles, veh_id)
        leader_current, follower_current = findclosest(current_vehicles, veh_id)

        # Calculate strategic incentive
        strategic = self._calculate_strategic(current_lane, desired_lane)

        # Calculate safety criterion
        is_safe = self._check_safety(veh_id, ego_speed, leader_target, follower_target)

        # Calculate speed gain incentive
        speed_gain = self._calculate_speed_gain(
            ego_speed, ego_max_speed, leader_current, leader_target
        )

        # Calculate keep right incentive
        keep_right = self._calculate_keep_right(current_lane, desired_lane)

        # Final decision based on weighted sum of incentives
        total_incentive = (
            STRATEGIC_PARAM * strategic
            + SPEEDGAIN_PARAM * speed_gain
            + KEEPRIGHT_PARAM * keep_right
        )

        return is_safe and total_incentive > 0

    def _calculate_strategic(self, current_lane: str, desired_lane: str) -> float:
        if desired_lane == current_lane:
            return 0.0
        return (
            1.0
            if int(desired_lane.split("_")[-1]) < int(current_lane.split("_")[-1])
            else -1.0
        )

    def _check_safety(
        self, ego_id: str, ego_speed: float, leader: str, follower: str
    ) -> bool:
        if leader:
            leader_speed = traci.vehicle.getSpeed(leader)
            gap_front = distance_between(ego_id, leader)
            if gap_front < MIN_SAFE_GAP:
                return False

        if follower:
            follower_speed = traci.vehicle.getSpeed(follower)
            gap_rear = abs(distance_between(follower, ego_id))
            if gap_rear < MIN_SAFE_GAP:
                return False

            # Check rear approach speed
            if follower_speed > ego_speed:
                required_gap = REACTION_TIME * (follower_speed - ego_speed)
                if gap_rear < required_gap:
                    return False

        return True

    def _calculate_speed_gain(
        self,
        ego_speed: float,
        max_speed: float,
        leader_current: str,
        leader_target: str,
    ) -> float:
        current_achievable = max_speed
        target_achievable = max_speed

        if leader_current:
            current_achievable = min(max_speed, traci.vehicle.getSpeed(leader_current))

        if leader_target:
            target_achievable = min(max_speed, traci.vehicle.getSpeed(leader_target))

        return target_achievable - current_achievable

    def _calculate_keep_right(self, current_lane: str, desired_lane: str) -> float:
        current_idx = int(current_lane.split("_")[-1])
        desired_idx = int(desired_lane.split("_")[-1])
        return 1.0 if desired_idx < current_idx else -1.0
