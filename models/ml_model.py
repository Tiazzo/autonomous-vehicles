import numpy as np
import joblib
import traci
from models.base_model import BaseDecisionModel
import os
import pandas as pd

def m_per_sec_2_f_per_sec(speed: float) -> float:
    return speed / 3.28084 # Go from m/s to ft/s



def distance_between(vehicle_1, vehicle_2):
    pos1 = traci.vehicle.getPosition(vehicle_1)
    pos2 = traci.vehicle.getPosition(vehicle_2)
    return (pos2[0] - pos1[0])

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

        # Vehicles in the target lane
        # This will give us a list of all the vehicles in the target lane in order of their position
        target_lane_vehicles = traci.lane.getLastStepVehicleIDs(desired_lane)
        print("target_lane_vechicles:",target_lane_vehicles)
        # identify the closest leading vehicle in the target lane
        closest_front_tl,closest_back_tl = findclosest(target_lane_vehicles, veh_id)
        print("closest_front_tl:",closest_front_tl, "closest_back_tl:",closest_back_tl)
        if target_lane_vehicles and closest_front_tl:
            v_tp = traci.vehicle.getSpeed(closest_front_tl)
            G_tp = distance_between(veh_id, closest_front_tl)
        else:
            v_tp = 1000     # values can not be infinite so we set it to a large number (for the ml model)
            G_tp = 1000

        # identify the following vehicle in the target lane 
        if target_lane_vehicles and closest_back_tl:
            G_tr = abs(distance_between(closest_back_tl, veh_id))
            v_tr = traci.vehicle.getSpeed(closest_back_tl)
        else:
            G_tr = 1000
            v_tr = 0.0

        # Current lane
        # identify the trailing vehicle in the current lane 
        current_lane_vehicles = traci.lane.getLastStepVehicleIDs(current_lane)
        print("current_lane_vechicles:",current_lane_vehicles)
      
        # find the index of the ego vehicle and add 1 to get the index of the preceding vehicle (car infront)
        if current_lane_vehicles.index('Ego') < (len(current_lane_vehicles)-1):
            preceding_vehicle = current_lane_vehicles[current_lane_vehicles.index('Ego') + 1]
            G_p = distance_between(preceding_vehicle, veh_id)
            v_p = traci.vehicle.getSpeed(preceding_vehicle)
        else:
            G_p = 1000
            v_p = 1000

      
        # Ego vehicle speed
        v_E = traci.vehicle.getSpeed(veh_id)
        # Get acceleration
        a_E = traci.vehicle.getAcceleration(veh_id)
        # Get the delta of speed between the ego vehicle and the leading vehicle
        delta_v_tp = v_E - v_tp
        delta_v_tr = v_E - v_tr

        ### Decision based model ###

        # Load the saved model and scaler
        base_path = os.path.dirname(os.path.abspath(__file__))  # Path of the current script
        model_path = os.path.join(base_path, 'ML', 'svm_model.pkl')
        scaler_path = os.path.join(base_path, 'ML', 'scaler.pkl')
        model = joblib.load(model_path)
        scaler = joblib.load(scaler_path)
       
        new_data = np.array([v_E,a_E,G_p,G_tr,G_tp,v_p,v_tr,v_tp,delta_v_tr,delta_v_tp])
        # convert the speeds and distances from m to ft
        for i in range(len(new_data)):
            new_data[i] = m_per_sec_2_f_per_sec(new_data[i])
        
        ## Preprocess the data 

        # define the column names as used during training
        feature_names = ['v_E', 'a_E', 'P', 'G_TR', 'G_TP', 'v_P', 'v_TR', 'v_TP', 'delta_v_TR', 'delta_v_TP']
  
        # convert to a pandas dataframe format
        new_data = pd.DataFrame([new_data], columns=feature_names)

        new_data_scaled = scaler.transform(new_data)
        # make predictions
        prediction = model.predict(new_data_scaled)

        print("Predictions:", prediction)
        should_change_lane = prediction[0]
        return should_change_lane
