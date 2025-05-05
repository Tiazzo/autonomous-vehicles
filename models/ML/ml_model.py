import numpy as np
import joblib
import traci
from models.base_model import BaseDecisionModel
# from sklearn.svm import SVC
# from sklearn.preprocessing import StandardScaler
# coefficient values (configurable)
A = 1
B = 1
C = 1
D = 1
E = 1
THETA = 0.5



def m_per_sec_2_f_per_sec(speed: float) -> float:
    return speed * 3.28084 # Go from m/s to ft/s



def distance_between(vehicle_1, vehicle_2):
    pos1 = traci.vehicle.getPosition(vehicle_1)
    pos2 = traci.vehicle.getPosition(vehicle_2)
    return abs(pos2[0] - pos1[0])

class ML(BaseDecisionModel):
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
        # Ego index
        ego_index = traci.vehicle.getLaneIndex(veh_id)
          
        # Vehicles in the target lane
        target_lane_vehicles = traci.lane.getLastStepVehicleIDs(desired_lane)
        
        # Identify the leading vehicle in the target lane
        if target_lane_vehicles:
            lead_vehicle = target_lane_vehicles[0]
            v_tp = traci.vehicle.getSpeed(lead_vehicle)
            G_tp = distance_between(veh_id, lead_vehicle)
        else:
            v_tp = 1000
            G_tp = float('inf')

        # Identify the trailing vehicle in the current lane
        current_lane_vehicles = traci.lane.getLastStepVehicleIDs(current_lane)
        if ego_index > 0:
            trailing_vehicle = current_lane_vehicles[ego_index + 1]
            G_p = distance_between(trailing_vehicle, veh_id)
            v_p = traci.vehicle.getSpeed(trailing_vehicle)
        else:
            G_p = float('inf')
            v_p = 1000

      
        # Ego vehicle speed
        v_E = traci.vehicle.getSpeed(veh_id)

        # safety metrics
        if target_lane_vehicles:
            trailing_tr = target_lane_vehicles[-1]
            G_tr = distance_between(trailing_tr, veh_id)
            v_tr = traci.vehicle.getSpeed(trailing_tr)
        else:
            G_tr = float('inf')
            v_tr = 0.0

        # Get acceleration
        a_E = traci.vehicle.getAcceleration(veh_id)
        
        # decision based on ml model
        # Load the saved model and scaler
        model = joblib.load('svm_model.pkl')
        scaler = joblib.load('scaler.pkl')
        new_data = np.array([v_E,a_E,G_p,G_tr,G_tp,v_p,v_tr,v_tp])
        # Convert the speed from m/s to ft/s
        for i in range(len(new_data)):
            new_data[i] = m_per_sec_2_f_per_sec(new_data[i])
        
        # Preprocess the new data
        new_data_scaled = scaler.transform(new_data)

        # Make predictions
        prediction = model.predict(new_data_scaled)

        print("Predictions:", prediction)
        should_change_lane = prediction
        return should_change_lane
