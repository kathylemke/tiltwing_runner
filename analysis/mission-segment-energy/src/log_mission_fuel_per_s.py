# log_power_all.py
#
# Usage: python3 log_power_all.py /path/to/cfg.json /path/to/log/
#  Reads the configuration JSON file and writes the results to the log directory
# Parameters:
#  /path/to/cfg.json: path to configuration JSON file
#  /path/to/log/: destination directory for log files
# Output:
#  Fuel burn in kg/s for each mission segment
#
# Written by Katherine Lemke
# Other contributors: Khoa Nguyen
#
# See the LICENSE file for the license

# import Python modules
import csv # csv
import sys # argv

# path to directory containing evtolpy package; use before deploying as package
sys.path.append('../../../evtol')
from aircraft import Aircraft
from mission import Mission

# parse script arguments
if len(sys.argv) == 3:
    cfg = sys.argv[1]
    log = sys.argv[2]
    if log[-1] != '/':
        log += '/'
else:
    print(
        'Usage: ' \
        'python3 log_power_all.py '
        '/path/to/cfg.json /path/to/log/'
    )
    exit()

# create aircraft object 
aircraft = Aircraft(cfg)

# data labels
mission_segment_labels = [\
 'Depart Taxi',
 'Hover Climb',
 'Transition Climb',
 'Depart Procedures',
 'Accelerate Climb',
 'Cruise Initial',
 'Decel to Drop Point',
 'Drop payload',
 'Accel from Drop Point',
 'Cruise return',
 'Decelerate Descend',
 'Arrive Procedures',
 'Transition Descend',
 'Hover Descend',
 'Arrive Taxi',
 'Reserve Hover Climb',
 'Reserve Transition Climb',
 'Reserve Accelerate Climb',
 'Reserve Cruise',
 'Reserve Decelerate Descend',
 'Reserve Transition Descend',
 'Reserve Hover Descend'
]

fuel_burn_per_s_vals = [
    aircraft.depart_taxi_kg_per_s,
    aircraft.hover_climb_kg_per_s,
    aircraft.trans_climb_kg_per_s,
    aircraft.depart_proc_kg_per_s,
    aircraft.accel_climb_kg_per_s,
    aircraft.cruise_initial_kg_per_s,
    
    aircraft.decel_descend_drop_point_kg_per_s,
    aircraft.drop_payload_kg_per_s,
    aircraft.accel_climb_drop_point_kg_per_s,
    aircraft.cruise_return_kg_per_s,
    aircraft.decel_descend_kg_per_s,
    aircraft.arrive_proc_kg_per_s,
    aircraft.trans_descend_kg_per_s,
    aircraft.hover_descend_kg_per_s,
    aircraft.arrive_taxi_kg_per_s,
    aircraft.reserve_hover_climb_kg_per_s,
    aircraft.reserve_trans_climb_kg_per_s,
    aircraft.reserve_accel_climb_kg_per_s,
    aircraft.reserve_cruise_kg_per_s,
    aircraft.reserve_decel_descend_kg_per_s,
    aircraft.reserve_trans_descend_kg_per_s,
    aircraft.reserve_hover_descend_kg_per_s,
]

# write the mission segment energy values to a CSV file in the log directory
with open(log+'fuel-cons-all.csv', 'w', newline='') as csvfile:
  csvwriter = csv.writer(csvfile)
  csvwriter.writerows([mission_segment_labels,fuel_burn_per_s_vals])

