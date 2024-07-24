# '''
# @file   launch_sim.py
# @brief  Automatic Installation Script for AIM softwares (On Clean Install)
# '''

# ======================================================================================
#                                   Description
# ======================================================================================

# '''
#     Note: Github SSH steps have to be performed before running this script. 
#     <<<Github RSA SSH key should be created and working>>>
#     <<<ssh-keygen>>>
#     @Requirements
#         python must be installed (ver >= 3.0) (ver <=3.10)
#         if python ver>=3.10:
#             make sure to run this command first:
#                 sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED
#                 (Instead of 3.11 put your python version there)
#         Make sure to use personal wifi
#     This script works under the assumption for a clean install. Existing software may
#     break the script.
# '''

# '''
# sudo apt install python3-pip

# '''
# ======================================================================================
#                                   Import Files
# ======================================================================================

import subprocess
from time import sleep
import json
import sys
import os

# ======================================================================================
#                               Constants / Global Variables
# ======================================================================================

kill_list=["ros2","runner","vectors","detect","synapse","ruby","xterm","parameter","image","robot","planner","odom","vel","waypoint","controller","async_","bt_nav","behavior_","smoother"]
input_file=open("models_config.json")
data=json.load(input_file)
curr_dir=os.getcwd()
curr_dir=curr_dir.split("AIM_2024")
curr_dir=curr_dir[0]
world_file_loc=""
world_file_input="Raceway_1_template.sdf"
world_file_output="Raceway_1.sdf"
def main():
    cmd_input=sys.argv
    input_len=len(cmd_input)
    print(cmd_input,input_len)
    str="pkill "
    if cmd_input[1].lower() == "start" or cmd_input[1].lower() == "stop":
        for i in kill_list:
            str="pkill "+i
            os.system(str)
    
    if cmd_input[1].lower() == "start":
        sleep(1)

        fileIn=open(world_file_input,"r")
        input_lines=fileIn.readlines()
        fileIn.close()
        fileOut=open(world_file_output,"w+")
        fileOut.writelines(input_lines)
        if(len(data)>0):
            for i in list(data.keys()):
                fileOut.write("\t<include>\n")
                string_input="\t\t<uri>models://"+data[i]["type"]+"</uri>\n"
                fileOut.write(string_input)
                string_input="\t\t<name>"+i+"</name>\n"
                fileOut.write(string_input)
                string_input="\t\t<pose>"+data[i]["pose"]+"</pose>\n"
                fileOut.write(string_input)
                fileOut.write("\t</include>\n")    
        fileOut.write("\t</world>\n")
        fileOut.write("</sdf>\n")
        fileOut.close()
        
        mv_wrld="cd "+curr_dir+"/AIM_2024_launcher && mv "+world_file_output+" "+curr_dir+"/src/dream_world/worlds/"+world_file_output
        print(mv_wrld)
        os.system(mv_wrld)
        
        os.system("cd ~/cognipilot/cranium && rm -rf build")
        os.system("cd ~/cognipilot/cranium && rm -rf install")
        os.system("cd ~/cognipilot/cranium && rm -rf log")
        os.system("cd ~/cognipilot/cranium && colcon build")
        sleep(1)
        os.system("source ~/cognipilot/cranium/install/setup.bash")
        os.system("cd ~/cognipilot/cranium && source ~/cognipilot/cranium/install/setup.bash")
        #sleep(1)
        #os.system("source ~/.bashrc")
        #os.system("tput reset && source ~/.profile")
        sleep(2)
        gz_subprocess=subprocess.Popen(["cd ~/cognipilot/cranium && ros2 launch b3rb_gz_bringup sil.launch.py world:=Raceway_1"],stdout=subprocess.DEVNULL,shell=True) #starting simulation
        sleep(1)
        ros_subprcoess1=subprocess.Popen(["cd ~/cognipilot/cranium && ros2 run b3rb_ros_line_follower vectors"],stdout=subprocess.DEVNULL,shell=True) #starting vectors node
        ros_subprcoess2=subprocess.Popen(["cd ~/cognipilot/cranium && ros2 run b3rb_ros_line_follower detect"],stdout=subprocess.DEVNULL,shell=True) #starting detect node
        ros_subprcoess3=subprocess.Popen(["cd ~/cognipilot/cranium && ros2 run b3rb_ros_line_follower runner"],stdout=subprocess.DEVNULL,shell=True) #starting runner node
        ros_subprcoess4=subprocess.Popen(["ros2 launch foxglove_bridge foxglove_bridge_launch.xml"],stdout=subprocess.DEVNULL,shell=True) #starting foxglove
        

    return

if __name__ == '__main__':   
    main() 
