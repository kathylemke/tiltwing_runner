# written by Katherine Lemke
# this file alls the initial MTOW guess
# comes from manual input or automated simulation
# then will run MTOW convergence to get a real MTOW for given payload
# next will run the fuel burn segments to show fuel consumed kg/s and total fuel consumed
# last will run the json export for simulation inputs
# all files will be saved in /log-all/ and /plt-all/ and named accordingly

import sys
import subprocess
import csv
import os

sys.path.append('../../evtol')
from aircraft import Aircraft
from mission import Mission

import json

if len(sys.argv) == 4:
    cfg = os.path.abspath(sys.argv[1]) # ensure that the path doesnt change even when using cwd
    log_dir = os.path.abspath(sys.argv[2])
    plt_dir = os.path.abspath(sys.argv[3])
    if log_dir[-1] != '/':
        log_dir += '/'
    if plt_dir[-1] != '/':
        plt_dir += '/'
else:
    print("Usage: python3 update_and_run.py /path/to/cfg.json /path/to/log/")
    exit()

# Get current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Path to the function folders
weight_dir = os.path.join(script_dir, "..", "mission-segment-weight", "src")
sim_dir = os.path.join(script_dir, "..", "simulation-outputs", "src")

# Absolute paths to scripts
log_script = os.path.abspath(os.path.join(weight_dir, "log_mtow_iteration.py"))
plt_script = os.path.abspath(os.path.join(weight_dir, "plt_mtow_iteration.py"))
sim_json_script = os.path.abspath(os.path.join(sim_dir, "sim_input.py"))

def run_mtow_convergence(cfg, log_dir, plt_dir):
    result_log = subprocess.run(
        ["python", log_script, cfg, log_dir],
        cwd=weight_dir,
        capture_output=True,
        text=True)
    if result_log.returncode != 0:
        print("Error running log_mtow_iteration.py")
        print(result_log.stderr)
    else:
        print("MTOW iteration log complete.")

    csv_path = os.path.join(log_dir, "mtow-iteration.csv")

    result_plt = subprocess.run(
        ["python", plt_script, csv_path, plt_dir],
        cwd=weight_dir,
        capture_output=True,
        text=True)
    if result_plt.returncode != 0:
        print("Error running plt_mtow_iteration.py")
        print(result_plt.stderr)
    else:
        print("MTOW iteration plt complete.")

def create_simulation_input(cfg, log_dir):
    results = subprocess.run(
        ["python", sim_json_script, cfg, log_dir],
        cwd=weight_dir,
        capture_output=True,
        text=True)
    if results.returncode != 0:
        print("Error running log_mtow_iteration.py")
        print(results.stderr)
    else:
        print("Generate sim inputs complete.")

run_mtow_convergence(cfg, log_dir, plt_dir)
# convergence run and json updated
# reload and define the aircraft with new json
create_simulation_input(cfg, log_dir)



 

    

    
    



    
