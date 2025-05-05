import traci
from models.base_model import BaseDecisionModel
import math

# Desired speed in km/h
V_SET_KMH = 40

# Default minimum safe gap (meters)
DEFAULT_G_TR_MIN = 20

# Speed advantage threshold for lane change (km/h)
SPEED_ADVANTAGE_THRESHOLD = 5

# Traffic density threshold for discouraging lane changes
TRAFFIC_DENSITY_THRESHOLD = 0.3

def kmh_2_ms(speed: float) -> float:
    return speed / 3.6

def adaptive_coefficients(v_E, traffic_density):
    A = 1 + 0.1 * traffic_density
    B = 1 + 0.05 * (kmh_2_ms(V_SET_KMH) - v_E)
    C = 1
    D = 1 + 0.1 * traffic_density
    E = 1
    return A, B, C, D, E

# def distance_between(vehicle_1, vehicle_2): # Should not be necessary as we are only interested in the x-axis
#     pos1 = traci.vehicle.getPosition(vehicle_1)
#     pos2 = traci.vehicle.getPosition(vehicle_2)
#     return math.sqrt((pos2[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)

def distance_between(vehicle_1, vehicle_2):
    pos1 = traci.vehicle.getPosition(vehicle_1)
    pos2 = traci.vehicle.getPosition(vehicle_2)
    return pos2[0] - pos1[0]

def findclosest(vehicles, ego):
    if vehicles:
        closest_front = None
        closest_back = None
        min_front_distance = float('inf')
        min_back_distance = float('-inf')
        for veh in vehicles:
            dis = distance_between(veh, ego)
            # compare distances to ego if greater than 0 then it is infront of ego vehicle
            if dis > 0 and dis < min_front_distance:
                min_front_distance = dis
                closest_front = veh
            if dis < 0 and dis > min_back_distance:
                min_back_distance = dis
                closest_back = veh
    else:
        closest_front = None
        closest_back = None
    return closest_front,closest_back

def dynamic_safe_gap(v_E, v_TR):
    return DEFAULT_G_TR_MIN + abs(v_E - v_TR) * 2  # Increase gap based on relative speed

class LiuImproved(BaseDecisionModel):
    def decide_lane_change(self, veh_id: str, current_lane: str, desired_lane: str) -> bool:
        v_set = kmh_2_ms(V_SET_KMH)
        v_E = traci.vehicle.getSpeed(veh_id)
        vehicle_type = traci.vehicle.getTypeID(veh_id) # dont think this is necessary

        traffic_density = traci.lane.getLastStepVehicleNumber(desired_lane) / max(traci.lane.getLength(desired_lane), 1)

        A, B, C, D, E = adaptive_coefficients(v_E, traffic_density)

        # vehicles in the target lane
        target_lane_vehicles = traci.lane.getLastStepVehicleIDs(desired_lane)
        
        # identify the closest front and back vehicles in the target lane
        closest_front_tl,closest_back_tl = findclosest(target_lane_vehicles, veh_id)
        if target_lane_vehicles and closest_front_tl:
            G_tp = distance_between(veh_id, closest_front_tl)
        else:
            G_tp = float('inf')

        # identify the following vehicle in the target lane 
        if target_lane_vehicles and closest_back_tl:
            G_tr = abs(distance_between(closest_back_tl, veh_id))
            v_TR = traci.vehicle.getSpeed(closest_back_tl)
        else:
            G_tr = float('inf')
            v_TR = 0.0


        # identify the preceding vehicle in the current lane 
        current_lane_vehicles = traci.lane.getLastStepVehicleIDs(current_lane)
    
        if current_lane_vehicles.index('Ego') < (len(current_lane_vehicles)-1):
            preceding_vehicle = current_lane_vehicles[current_lane_vehicles.index('Ego') + 1]
            G_p = distance_between(preceding_vehicle, veh_id)
            v_p = traci.vehicle.getSpeed(preceding_vehicle)
        else:
            G_p = float('inf')
            v_p = 1000

        # Benefit function
        v_ben = min(v_set - v_p, 0)
        f_ben = A * v_ben + B * (G_tp - G_p)

        # Tolerance function
        t_h = G_p / v_E if v_E > 0 else float('inf') # The float function here might cause the model to work badly
        f_tol = C * (G_p - v_E * t_h)

        # Safety function
        if target_lane_vehicles:
            trailing_tr = target_lane_vehicles[-1] # Again wrong indexing
            G_tr = distance_between(trailing_tr, veh_id)
            v_TR = traci.vehicle.getSpeed(trailing_tr)
        else:
            G_tr = float('inf')
            v_TR = 0

        # Dynamic safe gap calculation
        dynamic_G_TR_MIN = dynamic_safe_gap(v_E, v_TR)

        f_safety = D * max(G_tr - dynamic_G_TR_MIN, 0) + E * (v_E - v_TR)

        # Additional criteria for traffic density and speed advantage
        speed_advantage = v_p - v_E
        if traffic_density > TRAFFIC_DENSITY_THRESHOLD:
            return False

        if speed_advantage > kmh_2_ms(SPEED_ADVANTAGE_THRESHOLD):
            return True

        # Decision criteria with hysteresis to prevent oscillations
        should_change_lane = f_safety > 0 and (f_ben - 0.5 * f_tol) > 0

        return should_change_lane
