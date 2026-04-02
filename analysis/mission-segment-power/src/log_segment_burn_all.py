# log_power_profile_all.py
#
# Usage: python3 log_power_profile_all.py /path/to/cfg.json /path/to/log/
#  Reads the configuration JSON file and writes the results to the log directory
# Parameters:
#  /path/to/cfg.json: path to configuration JSON file
#  /path/to/log/: destination directory for log files
# Output:
#  Power profile of the whole mission
#
# Written by First Last
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
        'python3 log_power_profile_all.py '
        '/path/to/cfg.json /path/to/log/'
    )
    exit()

# create aircraft object 
aircraft = Aircraft(cfg)

# create mission object -
mission = Mission(cfg)

mission_segment_durations = [
    mission.depart_taxi_s,
    mission.hover_climb_s,
    mission.trans_climb_s,
    mission.accel_climb_s,
    mission.cruise_s,
    mission.decel_descend_s,
    mission.trans_descend_s,
    mission.hover_descend_s,
    mission.arrive_taxi_s,
    mission.reserve_hover_climb_s,
    mission.reserve_trans_climb_s,
    mission.reserve_accel_climb_s,
    mission.reserve_cruise_s,
    mission.reserve_decel_descend_s,
    mission.reserve_trans_descend_s,
    mission.reserve_hover_descend_s,
]

fuel_burn_values_kg_per_s = [
    aircraft.depart_taxi_kg_per_s,
    aircraft.hover_climb_kg_per_s,
    aircraft.trans_climb_kg_per_s,
    aircraft.accel_climb_kg_per_s,
    aircraft.reserve_cruise_kg_per_s,
    aircraft.decel_descend_kg_per_s,
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

time_steps = []
avg_fuel_burn = []
current_time = 0.0

for fuel_burn_kg_s, duration_s in zip(fuel_burn_values_kg_per_s, mission_segment_durations):
    for t in range(int(duration_s)):
        time_steps.append(current_time + t)
        avg_fuel_burn.append(fuel_burn_kg_s)
    current_time += duration_s

with open(log + 'fuel-burn-profile-all.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['time', 'fuel_burn_kg_s'])
    for t, p in zip(time_steps, avg_fuel_burn):
        csvwriter.writerow([f'{t:.3f}', f'{p:.6f}'])