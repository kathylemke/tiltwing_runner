# written by Katherine Lemke
# this file feeds straight into DLR simulation code


# import Python modules
import csv # csv
import sys # argv

sys.path.append('../../../evtol')
from aircraft import Aircraft
from mission import Mission
import json

# initialize script arguments
cfg = '' # path to configuration JSON file
log = '' # destination directory for log files

# parse script arguments
if len(sys.argv)==3:
  cfg = sys.argv[1]
  log = sys.argv[2]
  if log[-1] != '/':
    log += '/'
else:
  print(\
   'Usage: '\
   'python3 log_mission_segment_energy.py '\
   '/path/to/cfg.json /path/to/log/'\
  )
  exit()

ac_prop = Aircraft(cfg)
miss_prop = Mission(cfg)


def build_sim_input_json():
    return {
        "icon": "evtol.svg",
        "takeoff_landing_type": "vertipad",
        "autonomous": False,
        "mtom": ac_prop.max_takeoff_mass_kg,
        "empty_mass": ac_prop.empty_mass_kg,
        "payload": ac_prop.payload_kg,
        "flow_rate": (ac_prop.payload_kg/miss_prop.drop_payload_s),
        "can_scoop": True,
        "scooping_distance": 8, #m based on hose length
        "span": ac_prop.wingspan_m,
        "propulsion_input":{
            "architecture": "conventional",
            "total_propellant": ac_prop.total_mission_fuel_consumption,
            "reserve_propellant": ac_prop.total_reserve_fuel_consumption,
            "propellant_unit": "kg",
            "refueling_rate": 7.7, #defined by est
            "taxi_out_fc":  {
                str(ac_prop.max_takeoff_mass_kg): ac_prop.depart_taxi_kg_per_s, 
                str(ac_prop.mass_without_payload()): ac_prop.arrive_taxi_kg_per_s
            },
            "taxi_in_fc": {
                str(ac_prop.max_takeoff_mass_kg): ac_prop.depart_taxi_kg_per_s, 
                str(ac_prop.mass_without_payload()): ac_prop.arrive_taxi_kg_per_s
            },
            "takeoff_fc": ac_prop.hover_climb_kg_per_s,
            "transition_fc": ac_prop.trans_climb_kg_per_s,
            "retransition_fc": ac_prop.trans_descend_kg_per_s,
            "cruise_fc": {
                str(ac_prop.max_takeoff_mass_kg): ac_prop.cruise_initial_kg_per_s, 
                str(ac_prop.mass_without_payload()): ac_prop.cruise_return_kg_per_s
            },
            "cruise_climb_fc": {
                str(ac_prop.max_takeoff_mass_kg): ac_prop.accel_climb_kg_per_s, 
                str(ac_prop.mass_without_payload()): ac_prop.accel_climb_drop_point_kg_per_s
            },
            "cruise_descent_fc": {
                str(ac_prop.max_takeoff_mass_kg): ac_prop.decel_descend_drop_point_kg_per_s, 
                str(ac_prop.mass_without_payload()): ac_prop.decel_descend_kg_per_s
            },
            "landing_fc": ac_prop.hover_descend_kg_per_s,
            "loiter_fc": ac_prop.cruise_return_kg_per_s,
            "hover_fc": ac_prop.hover_fuel_consumption
        },

         "profile_parameters": { #so in the aircraft code there are more segments where you climb than this
             #simplicity of the simulation miss_prop phases omits depart and arrive procedures etc.
            "taxi_out_duration": miss_prop.depart_taxi_s,
            "taxi_in_duration": miss_prop.arrive_taxi_s,
            "transition_duration": miss_prop.trans_climb_s,
            "retransition_duration": miss_prop.trans_descend_s,
            "takeoff_altitude": miss_prop.hover_climb_s*miss_prop.hover_climb_avg_v_m_p_s, #altitude at which transition starts
            "takeoff_climb_rate": miss_prop.hover_climb_avg_v_m_p_s,
            "takeoff_ground_speed": 0, #as of rn only vert takeoff
            "cruise_altitude": miss_prop.cruise_alt_m,
            "cruise_speed": miss_prop.cruise_h_m_p_s,
            "cruise_climb_rate": miss_prop.accel_climb_v_m_p_s,
            "cruise_climb_ground_speed": miss_prop.accel_climb_avg_h_m_p_s,
            "cruise_descent_rate": miss_prop.decel_descend_v_m_p_s,
            "cruise_descent_ground_speed": miss_prop.decel_descend_avg_h_m_p_s,
            "landing_altitude": miss_prop.hover_descend_s*miss_prop.hover_descend_avg_v_m_p_s, #altitude at which transition to vert landing starts
            "landing_descent_rate": miss_prop.hover_descend_avg_v_m_p_s,
            "landing_ground_speed": 0, #again only vert motion
            "loiter_speed": ac_prop.stall_speed_m_p_s #will get there eventually
         }
    }

aircraft_data = build_sim_input_json()

with open(log + 'simulation-input.json', "w") as f:
    json.dump(aircraft_data, f, indent=4)

