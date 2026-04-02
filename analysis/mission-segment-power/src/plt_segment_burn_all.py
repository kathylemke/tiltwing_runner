# plt_power_profile_all.py
#
# Usage: python3 plt_power_profile_all.py /path/to/log.csv /path/to/plt/
#  Reads the log CSV file and saves the plot to the plt directory
#  Ensure that the Python virtual environment (venv) is enabled after running
#  setup_dependencies.sh: source p3-env/bin/activate
# Parameters:
#  /path/to/log.csv: path to log CSV file
#  /path/to/plt/: destination directory for plot files
# Output:
#  Plot for power profile of the whole mission
#
# Written by Katherine Lemke
# Other contributors: Khoa Nguyen
#
# See the LICENSE file for the license

#THis is kg/s plotted

# import Python modules
import csv
import matplotlib.pyplot as plt
import sys

# parse script arguments
if len(sys.argv) == 3:
    log_csv = sys.argv[1]
    out_dir = sys.argv[2]
    if out_dir[-1] != '/':
        out_dir += '/'
else:
    print(
        'Usage: '
        'python3 plt_power_profile_all.py '
        '/path/to/log.csv /path/to/plt/'
    )
    exit()

# read the log CSV file
time = []
power = []
with open(log_csv, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # skip header
    for row in csvreader:
        time.append(float(row[0]))
        power.append(float(row[1]))

# generate plot
fig, ax = plt.subplots(figsize=(12, 6), constrained_layout=True)

# segment labels
segment_labels = [
    'Depart Taxi',
    'Hover Climb',
    'Transition Climb',
    'Accelerate Climb',
    'Cruise',
    'Decelerate Descend',
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

# approximate segment boundaries using the time vector
segment_boundaries = [0]
for i in range(1, len(time)):
    if power[i] != power[i-1]:
        segment_boundaries.append(i)
segment_boundaries.append(len(time))  

# split into two parts (if available)
num_segments = max(0, len(segment_boundaries) - 1)
if num_segments == 0:
    raise ValueError('No segment boundaries found in log file, check input data.')

if num_segments < len(segment_labels):
    segment_labels = segment_labels[:num_segments]

# plotting by mission
if num_segments >= 9:
    split_idx = segment_boundaries[9]
    ax.step(time[:split_idx], power[:split_idx], where='post', color='g', label='Main Mission')
    ax.step(time[split_idx-1:], power[split_idx-1:], where='post', color='b', label='Reserve Mission')
    ax.axvline(x=time[split_idx], color='k', linestyle='--', linewidth=1)
else:
    split_idx = None
    ax.step(time, power, where='post', color='g', label='Main Mission')

# add labels
for i in range(num_segments):
    start_idx = segment_boundaries[i]
    end_idx = segment_boundaries[i+1]
    mid_time = (time[start_idx] + time[end_idx - 1]) / 2
    segment_power = power[start_idx:end_idx]
    if not segment_power:
        continue
    y_max = max(segment_power)
    y_offset = max(1e-6, y_max * 0.1)  # relative offset so label appears above data
    mid_power = y_max + y_offset

    ax.text(
        mid_time, mid_power,
        segment_labels[i],
        ha='center', va='bottom',
        fontsize=8,
        color=('g' if i < 9 else 'b')
    )

# format plot
ax.set_title('Fuel Burn Profile')
ax.set_xlabel('Flight Time (s)')
ax.set_ylabel('Fuel Burn rate (kg/s)')
ax.grid(True)
ax.legend()

# adjust layout and save
fig.tight_layout(pad=1.2)
fig.savefig(out_dir + 'fuel-burn-profile-all.pdf', format='pdf', bbox_inches='tight')
