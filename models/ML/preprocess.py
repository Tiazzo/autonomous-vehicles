# preprocess
import pandas as pd
import math
import numpy as np
import os
from pathlib import Path
# Load data using pandas
# List all .csv files
folder_path = Path("./data")
files = folder_path.glob("*.csv")
print(files)

for file in files:
    print(file.name)
    
    df = pd.read_csv('./data/' + file.name)
    # Cut down size of df for testing
    #df = df.iloc[:10000]

    # Filter out any row that has a v_Class of 1 or 3 as these are not cars
    # df = df[df['v_Class'] != 1]
    # df = df[df['v_Class'] != 3]
    # Drop collums that are not needed
    #check if Space_Hdwy is in the data
    if 'Space_Headway' in df.columns:
        df = df.rename(columns={'Space_Headway': 'Space_Hdwy', 'Time_Headway': 'Time_Hdwy'})
    
    df = df.drop(columns=['v_Length', 'v_Width','Following'])
   
    

    # Remove rows where Lane_ID is 6 - 8 as
    """
    Lane 1 is farthest left lane; lane 5 is farthest 
    right lane. Lane 6 is the auxiliary lane between Ventura Boulevard on-ramp and the Cahuenga 
    Boulevard off-ramp. Lane 7 is the on-ramp at Ventura Boulevard, and Lane 8 is the off-ramp at 
    Cahuenga Boulevard.
    """
    df = df[df['Lane_ID'] != 6]
    df = df[df['Lane_ID'] != 7]
    df = df[df['Lane_ID'] != 8]

# New plan: When lane change ID happends than mark the next row where the velocity is increased by 0.6096 m/s or more duirng the first 5 seconds of lane ID change this is START OF LANE CHANGE
# When this is found, calculate the P, G_TR, G_TP, v_P, v_TR, v_TP, Elapsed_Time and mark the row where the lane change happened and the row before
    df['P'] = 0.0
    df['G_TR'] = 0.0
    df['G_TP'] = 0.0
    df['v_P'] = 0.0
    df['v_TR'] = 0.0
    df['v_TP'] = 0.0
    # Like in novel paper mark the one before lane change and the one where lane change happens
    df['mark'] = 0 
    # Max Space Headway: 1321.55
    mean_headway = df['Space_Hdwy'].mean()
    headway_cap = mean_headway*2
    #df = df[df.index % 2 != 0].reset_index(drop=True)
    df = df.reset_index(drop=True)
    # Save the length of the dataframe
    og_len = len(df)
    def calc(x):
        not_faluty = True
        # Parameters of ego car
        lane_id = df['Lane_ID'].iloc[x]
        local_y = df['Local_Y'].iloc[x]
        global_time = (df['Global_Time'].iloc[x])
        df["Vehicle_ID"] = df["Vehicle_ID"].round(1)
        df["Global_Time"] = df["Global_Time"].round(1)

        ## Calualte P
        pred_id = df['Preceeding'].iloc[x]
        pred_id = int(pred_id)
        global_time = int(global_time)
        if pred_id != 0:
            # there is a car infront
            
            specific_vehicle_data = df[(df["Vehicle_ID"] ==  pred_id) & (df["Global_Time"] == global_time)]
    
            if specific_vehicle_data.empty:
                # We should mark that this pair is faulty
                not_faluty = False
                print("Faluoty at:",x, "for pred_id:",pred_id, "current_id:",df['Vehicle_ID'].iloc[x], "global_time:",global_time, "lane_id:",lane_id, "pred_lane_id",specific_vehicle_data['Lane_ID'])
                return not_faluty
                

            else:
                p = df['Space_Hdwy'].iloc[x]
                df.at[x, 'P'] = p
                v_p = specific_vehicle_data['v_Vel'].values[0]
                df.at[x, 'v_P'] = v_p
                
        else:
            return False
            v_p = df['v_Vel'].iloc[x] - 10
            df.at[x, 'P'] = headway_cap # Large number should be inf
            df.at[x, 'v_P'] = v_p
            
        
                
        ## Calculate G_TR and G_TP
        
        if lane_id != 0:
            next_lane_cars = df[
                (df["Lane_ID"] == lane_id - 1) &
                (df["Global_Time"] == global_time)]
           
            # Get y values for cars in next lane
            next_lane_front = next_lane_cars[next_lane_cars["Local_Y"] > local_y]
            next_lane_behind = next_lane_cars[next_lane_cars["Local_Y"] < local_y]
            # Get the car closest in the next lane
            # Front car
            if not next_lane_front.empty:
                front_car = next_lane_front.loc[next_lane_front["Local_Y"].idxmin()] # Returns id of the closest car infront
                v_tp = front_car["v_Vel"]
                
                g_tp = front_car["Local_Y"] - local_y  # Distance to the car ahead
                df.at[x, 'G_TP'] = round(g_tp,2)
                df.at[x, 'v_TP'] = v_tp
              
            else: 
                return False
                
                

            # Behind car  
            if not next_lane_behind.empty:
                behind_car = next_lane_behind.loc[next_lane_behind["Local_Y"].idxmax()] # Returns id of the closest car behind
                v_tr = behind_car["v_Vel"]
                
                g_tr = local_y - behind_car["Local_Y"]  # Distance to the car behind
                df.at[x, 'G_TR'] = round(g_tr,2)
                df.at[x, 'v_TR'] = v_tr
                
            else:
                return False
                
                
        
        return not_faluty

    lane_change_idx = 0
    start_of_lane_change = False
    one_per_id = False
    current_vel = 0
    for x in range(1, len(df)):
        ## Check if lane change happened
        if (
            df['Lane_ID'].iloc[x] < df['Lane_ID'].iloc[x-1] and 
            df['Vehicle_ID'].iloc[x] == df['Vehicle_ID'].iloc[x-1] and not start_of_lane_change and not one_per_id
        ):
            
            df.at[x,'mark'] = 1
            vel = df['v_Vel'].iloc[x]
            lane_change_idx = x
            start_time = df['Global_Time'].iloc[x]
            start_of_lane_change = True
            one_per_id = True
        elif (df['Vehicle_ID'].iloc[x] == df['Vehicle_ID'].iloc[x-1] and start_of_lane_change and one_per_id):
            
            current_vel = df['v_Vel'].iloc[x]
            timer = round(((df['Global_Time'].iloc[x] - start_time)*10e-4),1)
            
            diff_vel = current_vel - vel
            if diff_vel >= 1.9980315 and timer <= 5: # Feet instead of meters per seconds
                df.at[x,'mark'] = 2
                start_of_lane_change = False
                # Calculate  P, G_TR, G_TP, v_P, v_TR, v_TP for x and lane_change_idx
                
                not_faulty1 = calc(lane_change_idx)
                not_faulty2 = calc(x)
                if not_faulty1 and not_faulty2:
                    pass
                else:
                    df.at[lane_change_idx,'mark'] = 0
                    df.at[x,'mark'] = 0
                    one_per_id = False
                    start_of_lane_change = False
                
                lane_change_idx = 0
            else:
                vel = current_vel    
        else:
            one_per_id = False
            start_of_lane_change = False
            df.at[lane_change_idx,'mark'] = 0
            lane_change_idx = 0

            
    
    # find all index with mark != 0
    print(df[df['mark'] != 0])


    df = df.rename(columns={'v_Vel': 'v_E', 'v_Acc': 'a_E'})
    # Drop the extra rows that are produced
    df = df.drop(df.tail(len(df)-og_len).index)
    
    # REMOVE ALL LANE CAHNGES THAT INCREASE THE LANE ID - want to go from fast lane to slowest lane

    #save the dataframe to a new csv file
    update_file = 'update_' + file.name

    df.to_csv('./update_data/'+ update_file, index=False)


