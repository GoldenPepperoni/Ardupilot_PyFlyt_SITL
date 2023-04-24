import os
import subprocess
import time



if __name__ == "__main__":
    """
    Run all
    """
    param_file = "Prototype_Params_File.param"
    sim_file = "sim.py"

    curr_path = os.getcwd()

    
    sim_path = f"{curr_path}/{sim_file}"

    print("Starting PyFlyt simulator...")
    subprocess.Popen(["python3", sim_path])

    time.sleep(1)

    # Change directory to /ardupilot/ArduPlane/
    ardu_path = f"{curr_path}/../../ardupilot/ArduPlane/"
    os.chdir(ardu_path)

    # Build parameter file path 
    param_path = f"{curr_path}/../../Ardupilot_SITL/params/{param_file}"

    print("Starting Ardupilot SITL...")
    os.system(f"sim_vehicle.py -f JSON:192.168.0.135 --console --map --add-param-file={param_path} -w")





