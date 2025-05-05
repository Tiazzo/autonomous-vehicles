import traci
from models.base_model import BaseDecisionModel

# coefficient values (configurable)
A = 1
B = 1
C = 1
D = 1
E = 1
THETA = 0.5

G_tr_MIN = 15  # minimum safe gap for rear vehicle in target lane (meters)
V_SET_KMH = 40  # desired speed in km/h

def kmh_2_ms(speed: float) -> float:
    return speed / 3.6

def F_ben(v_ben, G_tp, G_p):
    return A * v_ben + B * (G_tp - G_p)

def F_tol(G_p, v_E, t_h):
    return C * (G_p - v_E * t_h)

def F_saf(g_tr, G_tr_MIN, v_E, v_tr):
    if g_tr >= G_tr_MIN:
        return D * (g_tr - G_tr_MIN) + E * (v_E - v_tr)
    return -float("inf")

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

class Liu(BaseDecisionModel):
    def decide_lane_change(self, veh_id: str, current_lane: str, desired_lane: str) -> bool:
        """
        Determines whether the vehicle should change lanes based on Liu et al. model.

        Args:
            veh_id (str): The ID of the vehicle.
            current_lane (str): The current lane ID of the vehicle.
            desired_lane (str): The target lane ID for the vehicle.

        Returns:
            bool: True if the vehicle decides to change lanes, False otherwise.
        """
        # benefit metrics
        v_set = kmh_2_ms(V_SET_KMH)  # Desired speed in m/s

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
        ### Identify the trailing vehicle in the current lane ###
        current_lane_vehicles = traci.lane.getLastStepVehicleIDs(current_lane)
    
        if current_lane_vehicles.index('Ego') < (len(current_lane_vehicles)-1):
            preceding_vehicle = current_lane_vehicles[current_lane_vehicles.index('Ego') + 1]
            G_p = distance_between(preceding_vehicle, veh_id)
            v_p = traci.vehicle.getSpeed(preceding_vehicle)
        else:
            G_p = float('inf')
            v_p = 1000


        v_ben = min(v_set - v_p, 0)  # Adjust logic as needed
        f_ben = F_ben(v_ben, G_tp, G_p)

        # tolerance metrics
        v_E = traci.vehicle.getSpeed(veh_id)
        t_h = G_p / v_E if v_E > 0 else G_p / kmh_2_ms(1) # can not be else: infinity
        f_tol = F_tol(G_p, v_E, t_h)

        # safety metric
        f_safety = F_saf(G_tr, G_tr_MIN, v_E, v_TR)

        # print all metrics in one row
        # decision based on Liu et al. criteria
        should_change_lane = f_safety > 0 and (f_ben - THETA * f_tol) > 0

        print(f"v_ben: {v_ben:.2f}, f_ben: {f_ben:.2f}, f_tol: {f_tol:.2f}, f_safety: {f_safety:.2f}, v_E: {v_E:.2f}, G_p: {G_p:.2f}, G_tp: {G_tp:.2f}, G_tr: {G_tr:.2f}, v_TR: {v_TR:.2f}, v_p: {v_p:.2f}, t_h: {t_h:.2f}, should_change_lane: {should_change_lane}")

        return should_change_lane
