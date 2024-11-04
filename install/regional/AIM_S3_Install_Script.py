# '''
# @file   install.py
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
import time
import streamlit as st
import json
import threading
import csv
import os

# ======================================================================================
#                               Constants / Global Variables
# ======================================================================================

username = ' '
password = ''
cwd = os.getcwd()
progress_file_path = os.path.join(cwd, 'AIM_S3_Install_Script_Progress.txt')
logs = os.path.join(cwd,'AIM_S3_Install_Script_Logs.txt')
SET_username = False
SET_password = False
normal_return = 0
wait_time = 1
set_1 = True
set_2 = True
set_3 = True
set_4 = True
set_5 = True
set_6 = True
set_1_r = 0
set_2_r = 0
set_3_r = 0
set_4_r = 0
set_5_r = 0
set_6_r = 0
details = {'username': username, 'password': '', 'SET_username': SET_username, 'SET_password': SET_password, 'set_1': set_1, 'set_2': set_2, 'set_3': set_3, 'set_4': set_4, 'set_5': set_5, 'set_6': set_6, 'set_1_r': set_1_r, 'set_2_r': set_2_r, 'set_3_r':set_3_r, 'set_4_r': set_4_r, 'set_5_r':set_5_r, 'set_6_r':set_6_r}
detailsstr = [f",{username},{SET_username},{SET_password},{set_1},{set_2},{set_3},{set_4},{set_5},{set_6},{set_1_r},{set_2_r},{set_3_r},{set_4_r},{set_5_r},{set_6_r},"]

# ======================================================================================
#                                  Function Definitions
# ======================================================================================

def run_code(command, args = ''):
    # Run the script file
    result = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,text = True)
    stdout = ''
    stderr = ''
    if args == '':
        stdout, stderr = result.communicate()
    else:
        stdout, stderr = result.communicate(input = args)
    return [result.returncode, stdout, stderr]

def run_code_2(command, args = ''):
    # Run the script file
    result = subprocess.Popen(command, shell = True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,text = True)
    stdout = ''
    stderr = ''
    if args == '':
        stdout, stderr = result.communicate()
    else:
        stdout, stderr = result.communicate(input = args)
    return [result.returncode, stdout, stderr]
  
# ======================================================================================
#                                   Set Status Variables 
# ======================================================================================

def getSV():
    global username
    global password
    global SET_username
    global SET_password
    global set_1 
    global set_2 
    global set_3 
    global set_4 
    global set_5
    global set_6
    global set_1_r
    global set_2_r 
    global set_3_r 
    global set_4_r 
    global set_5_r
    global set_6_r
    global progress_file_path
    global details
    global detailsstr
    cwd = os.getcwd()
    progress_file_path = os.path.join(cwd, 'AIM_S3_Install_Script_Progress.txt')
    data = ''
    with open(progress_file_path, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',', quotechar='"',quoting=csv.QUOTE_ALL)
        for row in reader:
            data = str(row).split(",")
            del data[0]
            del data[-1]
    #print(data)
    #print(len(data))
    
    username = data[0]
    
    if "True" in data[1]:
        SET_username = True
    else:
        SET_username = False
    
    if "True" in data[2]:
        SET_password = True
    else:
        SET_password = False

    if "True" in data[3]:
        set_1 = True
    else:
        set_1 = False
        
    if "True" in data[4]:
        set_2 = True
    else:
        set_2 = False
        
    if "True" in data[5]:
        set_3 = True
    else:
        set_3 = False
        
    if "True" in data[6]:
        set_4 = True
    else:
        set_4 = False
        
    if "True" in data[7]:
        set_5 = True
    else:
        set_5 = False
    if "True" in data[8]:
        set_6 = True
    else:
        set_6 = False
    
    set_1_r = int(data[9])
    set_2_r = int(data[10])
    set_3_r = int(data[11])
    set_4_r = int(data[12])
    set_5_r = int(data[13])
    set_6_r = int(data[14])

def setSV(name='', value=''):
    global username
    global password
    global SET_username
    global SET_password
    global set_1 
    global set_2 
    global set_3 
    global set_4 
    global set_5
    global set_6
    global set_1_r
    global set_2_r 
    global set_3_r 
    global set_4_r 
    global set_5_r
    global set_6_r
    global progress_file_path
    global details
    global detailsstr
    details = {'username': username, 'password': '', 'SET_username': SET_username, 'SET_password': SET_password, 'set_1': set_1, 'set_2': set_2, 'set_3': set_3, 'set_4': set_4, 'set_5': set_5, 'set_6': set_6,'set_1_r': set_1_r, 'set_2_r': set_2_r, 'set_3_r':set_3_r, 'set_4_r': set_4_r, 'set_5_r':set_5_r , 'set_6_r':set_6_r}

    name = (str)(name)
    if name=='' and value == '':
        pass
    else:
        details[name] = value

    detailsstr = [f",{details['username']},{details['SET_username']},{details['SET_password']},{details['set_1']},{details['set_2']},{details['set_3']},{details['set_4']},{details['set_5']},{details['set_6']},{details['set_1_r']},{details['set_2_r']},{details['set_3_r']},{details['set_4_r']},{details['set_5_r']},{details['set_6_r']},"]
    #print(detailsstr)
    with open(progress_file_path, 'w') as convert_file: 
        convert_file.write(json.dumps(str(detailsstr)))
    getSV()
    
   
# ======================================================================================
#                               Backend Functions (Commands)
# ======================================================================================

def SET_1_COMMANDS_MAIN():
    global set_1_r
    # setSV('set_1_r', 2)
    # getSV()
    t1 = threading.Thread(target=SET_1_COMMANDS)
    t1.start()
    t1.join()
    if set_1_r == 1:
        setSV('set_2',True)
        setSV('set_1',False)
        st.toast("SET 1 SUCCESSFUL! Please RESTART your Device!")

def SET_1_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time
    
    print('SET 1 START')
    print('Setting locales')

    ret = run_code_2(f"sudo apt update && sudo apt install locales |  tee logs/s1/c1.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('...')

    ret = run_code_2(f"sudo locale-gen en_US en_US.UTF-8 |  tee logs/s1/c2.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Updating locales')

    ret = run_code_2(f"sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 |  tee logs/s1/c3.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Exporting locales')

    ret = run_code_2(f"export LANG=en_US.UTF-8 |  tee logs/s1/c4.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('installing sowftare-properties-common')

    ret = run_code_2(f"sudo apt install software-properties-common |  tee logs/s1/c5.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('adding to apt repositories')

    ret = run_code_2(f"sudo add-apt-repository universe |  tee logs/s1/c6.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('updating')

    ret = run_code_2(f"sudo apt update && sudo apt install curl -y |  tee logs/s1/c7.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Setting up to download ROS')

    ret = run_code_2(f"sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg |  tee logs/s1/c8.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('...')

    ret = run_code_2(f'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null')
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Getting ROS')

    ret = run_code_2(f"sudo apt update |  tee logs/s1/c9.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('...')

    ret = run_code_2(f"sudo apt upgrade |  tee logs/s1/c10.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Installing ros-humble-desktop')

    ret = run_code_2(f"sudo apt install ros-humble-desktop |  tee logs/s1/c11.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Installing ros-humble-base')

    ret = run_code_2(f"sudo apt install ros-humble-ros-base |  tee logs/s1/c12.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Installing ros-dev-tools')

    ret = run_code_2(f"sudo apt install ros-dev-tools |  tee logs/s1/c13.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return
    print('Sourcing')

    ret = run_code_2(f"source /opt/ros/humble/setup.bash |  tee logs/s1/c14.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_1_r', -1)
        return

    
    setSV('set_1_r', 1)
    print('SET 1 END')
    return
    
def SET_2_COMMANDS_MAIN():
    global set_2_r
    t1 = threading.Thread(target=SET_2_COMMANDS)
    t1.start()
    t1.join()
    if set_2_r == 1:
        setSV('set_3',True)
        setSV('set_2',False)
        st.toast("SET 2 SUCCESSFUL! Please RESTART your Device!")
    
def SET_2_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time
    
    print('SET 2 START')
    print('Updating')
    ret = run_code_2(f"sudo -S apt-get update | tee logs/s2/c1.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    
    print('1Installing gnome-terminal')
    ret = run_code_2(f"sudo apt install gnome-terminal |  tee logs/s2/c2.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    
    print('Updating')
    ret = run_code_2(f"sudo -S apt-get update |  tee logs/s2/c3.txt2", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    
    print('Installing dependencies')
    ret = run_code_2(f"sudo -S apt-get install git wget -y |  tee logs/s2/c4.txt", password)
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    
    print('Creating Directories')
    ret = run_code_2(f"mkdir -p ~/cognipilot/installer |  tee logs/s2/c5.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    
    print('updating script')
    ret = run_code_2(f"wget -O ~/cognipilot/installer/install_cognipilot.sh https://raw.githubusercontent.com/CogniPilot/helmet/main/install/install_cognipilot.sh |  tee logs/s2/c6.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    

    ret = run_code_2(f"chmod a+x ~/cognipilot/installer/install_cognipilot.sh |  tee logs/s2/c7.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_2_r', -1)
        return
    count = 0
    flag = 0
    while(count<10):
        print('Installing cognipilot')
        count += 1
        result = run_code(['gnome-terminal', '--', 'bash', '-c', "/bin/bash ~/cognipilot/installer/install_cognipilot.sh |  tee logs/s2/c8.txt"])
        if result[0] == normal_return:
            flag = 1
            break
        print('failed - retrying')
    if flag == 0:
        setSV('set_2_r', -1)
        return
    
    flag = 0
    count = 0
    while (count < 10):
        print('Sourcing')
        count += 1
        result_2 = run_code(['gnome-terminal', '--', 'bash', '-c', "source ~/.bashrc |  tee logs/s2/c9.txt"])
        if result_2[0] == normal_return:
            flag = 1
            break
        print('Sourcing failed - retrying')
    if flag == 0:
        setSV('set_2_r', -1)
        return
    
    setSV('set_2_r', 1)
    print('SET 2 END')
    return

def SET_3_COMMANDS_MAIN():
    global set_3_r
    t1 = threading.Thread(target=SET_3_COMMANDS)
    t1.start()
    t1.join()
    if set_3_r == 1:
        setSV('set_4',True)
        setSV('set_3',False)
        st.toast("SET 3 SUCCESSFUL! Please RESTART your Device!")

def SET_3_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time
    
    print('SET 3 START')

    flag = 0
    count = 0
    while (count < 10):
        print('Building workspace')
        count += 1
        result_2 = run_code_2("build_workspace  |  tee logs/s3/c1.txt", "y\n1\n")
        if result_2[0] == normal_return:
            flag = 1
            break
        print('failed - retrying')
    time.sleep(wait_time)
    if flag == 0:
        setSV('set_3_r', -1)
        return

    flag = 0
    count = 0
    while (count < 10):
        print('1')
        count += 1
        result_2 = run_code(['gnome-terminal', '--', 'bash', '-c', "source ~/.bashrc |  tee logs/s3/c2.txt"])
        if result_2[0] == normal_return:
            flag = 1
            break
        print('sourcing failed - retrying')
    if flag == 0:
        setSV('set_3_r', -1)
        return
    
    setSV('set_3_r', 1)
    print('SET 3 END')
    return

def SET_4_COMMANDS_MAIN():
    global set_4_r
    t1 = threading.Thread(target=SET_4_COMMANDS)
    t1.start()
    t1.join()
    if set_4_r == 1:
        setSV('set_5',True)
        setSV('set_4',False)
        st.toast("SET 4 SUCCESSFUL! Please RESTART your Device!")

def SET_4_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time

    print('SET 4 START')

    ret = run_code_2("cd ~/cognipilot/ws/cerebri  |  tee logs/s4/c1.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_4_r', -1)
        return
    
    print('Pulling latest')
    ret = run_code_2("cd ~/cognipilot/ws/cerebri; git pull  |  tee logs/s4/c2.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_4_r', -1)
        return
    
    print('Installing west')
    ret = run_code_2("cd ~/cognipilot/ws/cerebri; sudo -S pip install west  |  tee logs/s4/c3.txt",password)
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_4_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/ws/cerebri  |  tee logs/s4/c4.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_4_r', -1)
        return
    
    print('Setting and running west')
    flag = 0
    count = 0
    while (count < 10):
        print('4')
        count += 1
        result_2 = run_code_2("cd ~/cognipilot/ws/cerebri ; west update |  tee logs/s4/c5.txt")
        if result_2[0] == normal_return  or result_2[0]==1:
            flag = 1
            break
        print('4 failed - retrying')
    time.sleep(wait_time)
    if flag == 0:
        setSV('set_4_r', -1)
        return
    

    ret = run_code_2("cd ~/cognipilot/ws/cerebri |  tee logs/s4/c6.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_4_r', -1)
        return
    
    flag = 0
    count = 0
    while (count < 10):
        print('Building canhubk3 > cerebri')
        count += 1
        result_2 = run_code_2("cd ~/cognipilot/ws/cerebri ; west build -b mr_canhubk3 app/b3rb -p |  tee logs/s4/c7.txt")
        if result_2[0] == normal_return or result_2[0]==1:
            flag = 1
            break
        print('failed - retrying')
    time.sleep(wait_time)
    if flag == 0:
        setSV('set_4_r', -1)
        return
    
    ret = run_code_2("cd ~/cognipilot/ws/cerebri/app/b3rb |  tee logs/s4/c8.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_4_r', -1)
        return
    
    flag = 0
    count = 0
    while (count < 10):
        print('Building canhubk3 > cerebri > app > b3rb')
        count += 1
        result_2 = run_code_2("cd ~/cognipilot/ws/cerebri/app/b3rb ; west build -b mr_canhubk3 -p |  tee logs/s4/c9.txt")
        if result_2[0] == normal_return or result_2[0] ==1:
            flag = 1
            break
        print('failed - retrying')
    time.sleep(wait_time)
    if flag == 0:
        setSV('set_4_r', -1)
        return
    
    print('DONE')
    setSV('set_4_r', 1)
    print('SET 4 END')
    return

def SET_5_COMMANDS_MAIN():
    global set_5_r
    t1 = threading.Thread(target=SET_5_COMMANDS)
    t1.start()
    t1.join()
    if set_5_r == 1:
        setSV('set_6',True)
        setSV('set_5',False)
        st.toast("SET 5 SUCCESSFUL! Please RESTART your Device!")

def SET_5_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time
    
    print('SET 5 START')

    ret = run_code_2("cd ~/cognipilot  |  tee logs/s5/c1.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    ret = run_code_2("cd ~/cognipilot  ; chmod +x update_repos_native.sh  |  tee logs/s5/c2.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_5_r', -1)
        return

    print('Running update_repo script')
    ret = run_code_2("cd ~/cognipilot  ; ./update_repos_native.sh  |  tee logs/s5/c3.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/cranium  |  tee logs/s5/c4.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    print('Building Cranium')
    ret = run_code_2("cd ~/cognipilot/cranium ; colcon build --symlink-install  |  tee logs/s5/c5.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/cranium/  |  tee logs/s5/c6.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    print('Sourcing')
    ret = run_code_2("cd ~/cognipilot/cranium/ ; source install/setup.bash  |  tee logs/s5/c7.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/electrode/  |  tee logs/s5/c8.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    print('Building electrode')
    ret = run_code_2("cd ~/cognipilot/electrode/ ; colcon build --symlink-install  |  tee logs/s5/c9.txt")
    print(ret)
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/electrode/  |  tee logs/s5/c10.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    print('Sourcing')
    ret = run_code_2("cd ~/cognipilot/electrode/ ; source install/setup.bash  |  tee logs/s5/c11.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return

    print('Setting west')
    ret = run_code_2("cd ~/cognipilot/ws/cerebri  |  tee logs/s5/c12.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    print('Running west update')
    ret = run_code_2("cd ~/cognipilot/ws/cerebri  ; west update  |  tee logs/s5/c13.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return

    print('Building native_sim')
    ret = run_code_2("cd ~/cognipilot/ws/cerebri  ; west build -b native_sim app/b3rb/ -p -t install  |  tee logs/s5/c14.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_5_r', -1)
        return

    print('Sourcing')
    ret = run_code_2("source ~/.bashrc  |  tee logs/s5/c15.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_5_r', -1)
        return

    setSV('set_5_r', 1)
    print('SET 5 END')
    return

def SET_6_COMMANDS_MAIN():
    global set_6_r
    t1 = threading.Thread(target=SET_6_COMMANDS)
    t1.start()
    t1.join()
    if set_6_r == 1:
        setSV('set_6',False)
        st.toast("SET 6 SUCCESSFUL! Please RESTART your Device!")

def SET_6_COMMANDS():
    global username
    global password
    global normal_return
    global wait_time
    
    print('SET 6 START')
    
    print('Installing libraries')
    ret = run_code_2("pip install cython | tee logs/s6/c1.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_6_r', -1)
        return
    print('...')
    ret = run_code_2("pip install opencv-python | tee logs/s6/c2.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_6_r', -1)
        return
    print('Cloning line_follower from NXP_AIM_INDIA')
    ret = run_code_2("cd ~/Downloads/ ; git clone git@github.com:NXPHoverGames/NXP_AIM_INDIA_2024.git | tee logs/s6/c3.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_6_r', -1)
        return

    print('Creating b3rb_ros_line_follower Directory')
    ret = run_code_2("rm -r ~/cognipilot/cranium/src/b3rb_ros_line_follower | tee logs/s6/c4.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    print('Adding line_follower demo code')
    ret = run_code_2("cp -r ~/Downloads/NXP_AIM_INDIA_2024/b3rb_ros_line_follower/ ~/cognipilot/cranium/src/ | tee logs/s6/c5.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/cranium/src  | tee logs/s6/c6.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    print('Updating dream_world')
    ret = run_code_2("cd ~/cognipilot/cranium/src  ; rm -rf dream_world | tee logs/s6/c7.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0]!=1:
        setSV('set_6_r', -1)
        return

    print('Updating synapse_msgs')
    ret = run_code_2("cd ~/cognipilot/cranium/src  ; rm -rf synapse_msgs | tee logs/s6/c8.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] !=1:
        setSV('set_6_r', -1)
        return

 
    ret = run_code_2("cd ~/cognipilot/cranium/src  ; git clone git@github.com:NXPHoverGames/synapse_msgs.git | tee logs/s6/c9.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_6_r', -1)
        return

    print('synapse_msgs Updated')
    ret = run_code_2("cd ~/cognipilot/cranium/src  ; git clone git@github.com:NXPHoverGames/dream_world.git | tee logs/s6/c10.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_6_r', -1)
        return

    print('dream_world updated')
    ret = run_code_2("cd ~/cognipilot/cranium/src/synapse_msgs | tee logs/s6/c11.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    print('checking line_follower git branches')
    ret = run_code_2("cd ~/cognipilot/cranium/src/synapse_msgs ; git checkout b3rb_ros_line_follower | tee logs/s6/c12.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_6_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/cranium/src/dream_world | tee logs/s6/c13.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    print('checking track git branch')
    ret = run_code_2("cd ~/cognipilot/cranium/src/dream_world ; git checkout aim_track | tee logs/s6/c14.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return and ret[0] != 1:
        setSV('set_6_r', -1)
        return


    ret = run_code_2("cd ~/cognipilot/cranium/ | tee logs/s6/c15.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    flag = 0
    count = 0
    while (count < 10):
        print('Trying to build at ~/cognipilot/cranium/')
        count += 1
        result_2 = run_code_2("cd ~/cognipilot/cranium/ ; colcon build | tee logs/s6/c16.txt")
        print(result_2)
        if result_2[0] == normal_return or result_2[0] ==1:
            flag = 1
            break
        print('build failed - retrying')
    if flag == 0:
        setSV('set_6_r', -1)
        return

    print('sourcing')
    ret = run_code_2("source ~/.bashrc | tee logs/s6/c17.txt")
    time.sleep(wait_time)
    if ret[0] != normal_return:
        setSV('set_6_r', -1)
        return

    setSV('set_6_r', 1)
    print('SET 6 END')
    return

def RESET():
    global username
    global password
    global SET_username
    global SET_password
    global set_1 
    global set_2 
    global set_3 
    global set_4 
    global set_5
    global set_6
    global set_1_r
    global set_2_r 
    global set_3_r 
    global set_4_r 
    global set_5_r
    global set_6_r
    global progress_file_path
    global details
    global detailsstr
    SET_username = False
    SET_password = False
    set_1 = True
    set_2 = False
    set_3 = False
    set_4 = False
    set_5 = False
    set_6 = False
    set_1_r = 0
    set_2_r = 0
    set_3_r = 0
    set_4_r = 0
    set_5_r = 0
    set_6_r = 0
    details = {'username': username, 'password': '', 'SET_username': SET_username, 'SET_password': SET_password, 'set_1': set_1, 'set_2': set_2, 'set_3': set_3, 'set_4': set_4, 'set_5': set_5, 'set_6':set_6, 'set_1_r': set_1_r, 'set_2_r': set_2_r, 'set_3_r':set_3_r, 'set_4_r': set_4_r, 'set_5_r':set_5_r, 'set_6_r':set_6_r }
    detailsstr = [f",{details['username']},{details['SET_username']},{details['SET_password']},{details['set_1']},{details['set_2']},{details['set_3']},{details['set_4']},{details['set_5']},{details['set_6']},{details['set_1_r']},{details['set_2_r']},{details['set_3_r']},{details['set_4_r']},{details['set_5_r']},{details['set_6_r']},"]
    #print(detailsstr)
    with open(progress_file_path, 'w') as convert_file: 
        convert_file.write(json.dumps(str(detailsstr)))
    getSV()
    
def OPEN_ALL():
    global username
    global password
    global SET_username
    global SET_password
    global set_1 
    global set_2 
    global set_3 
    global set_4 
    global set_5
    global set_1_r
    global set_2_r 
    global set_3_r 
    global set_4_r 
    global set_5_r
    global set_6_r
    global progress_file_path
    global details
    global detailsstr
    SET_username = False
    SET_password = False
    set_1 = True
    set_2 = True
    set_3 = True
    set_4 = True
    set_5 = True
    set_6 = True
    details = {'username': username, 'password': '', 'SET_username': SET_username, 'SET_password': SET_password, 'set_1': set_1, 'set_2': set_2, 'set_3': set_3, 'set_4': set_4, 'set_5': set_5, 'set_6':set_6, 'set_1_r': set_1_r, 'set_2_r': set_2_r, 'set_3_r':set_3_r, 'set_4_r': set_4_r, 'set_5_r':set_5_r, 'set_6_r':set_6_r }
    detailsstr = [f",{details['username']},{details['SET_username']},{details['SET_password']},{details['set_1']},{details['set_2']},{details['set_3']},{details['set_4']},{details['set_5']},{details['set_6']},{details['set_1_r']},{details['set_2_r']},{details['set_3_r']},{details['set_4_r']},{details['set_5_r']},{details['set_6_r']},"]
    #print(detailsstr)
    with open(progress_file_path, 'w') as convert_file: 
        convert_file.write(json.dumps(str(detailsstr)))
    getSV()
    
def CLOSE_ALL():
    global username
    global password
    global SET_username
    global SET_password
    global set_1 
    global set_2 
    global set_3 
    global set_4 
    global set_5
    global set_6
    global set_1_r
    global set_2_r 
    global set_3_r 
    global set_4_r 
    global set_5_r
    global set_6_r
    global progress_file_path
    global details
    global detailsstr
    SET_username = False
    SET_password = False
    set_1 = False
    set_2 = False
    set_3 = False
    set_4 = False
    set_5 = False
    set_6 = False
    details = {'username': username, 'password': '', 'SET_username': SET_username, 'SET_password': SET_password, 'set_1': set_1, 'set_2': set_2, 'set_3': set_3, 'set_4': set_4, 'set_5': set_5, 'set_6':set_6, 'set_1_r': set_1_r, 'set_2_r': set_2_r, 'set_3_r':set_3_r, 'set_4_r': set_4_r, 'set_5_r':set_5_r, 'set_6_r':set_6_r }
    detailsstr = [f",{details['username']},{details['SET_username']},{details['SET_password']},{details['set_1']},{details['set_2']},{details['set_3']},{details['set_4']},{details['set_5']},{details['set_6']},{details['set_1_r']},{details['set_2_r']},{details['set_3_r']},{details['set_4_r']},{details['set_5_r']},{details['set_6_r']},"]
    #print(detailsstr)
    with open(progress_file_path, 'w') as convert_file: 
        convert_file.write(json.dumps(str(detailsstr)))
    getSV()

# ======================================================================================
#                                      Frontend / UI
# ======================================================================================
           
def main(): 
    global username
    global password
    st.title('AIM Installation Script')
    st.text('Follow the following steps for installation:')
    st.text('1. Enter device username and password')
    st.text('2. Run one SET of commands and wait for that set to execute')
    st.text('3. Refer to the Installation Guide for steps to follow during exection of a SET')
    st.text('4. Restart your device after running a SET ')
    col1, col2 = st.columns(2)
    with col1:
        username = st.text_input('Username', disabled = SET_username, value = username, help = 'Enter your machine username')
    with col2:
        password = st.text_input('Password', disabled = SET_password, value = password, help = 'Enter your machine password', type = 'password')
    main_area_4 = st.empty()

    c1c1, c1c2 = st.columns([5,10])
    c2c1, c2c2 = st.columns([5,10])
    c3c1, c3c2 = st.columns([5,10])
    c4c1, c4c2 = st.columns([5,10])
    c5c1, c5c2 = st.columns([5,10])
    c6c1, c6c2 = st.columns([5,10])
    
    col11, col21 = st.columns(2)
    with col11:
        reset_button = st.button('RESET PROGRESS', on_click=RESET)
    with col21:
        open_all = st.button('OPEN ALL', on_click=OPEN_ALL)

    with c1c1:
        set_1_button = st.button('Start Installation (SET 1)', disabled=not set_1, on_click=SET_1_COMMANDS_MAIN)
    with c1c2:
        set_1_result = st.empty()
        if set_1_r == 0:
            set_1_result.empty()
        elif set_1_r == 1:
            set_1_result.success("SET 1 Commands ran successfully")
        elif set_1_r == -1:
            set_1_result.error("ERROR Running SET 1 Commands. Please Procees Manually")
        else:
            set_1_result.success("RUNNING SET 1. Please Wait")
            CLOSE_ALL()

    with c2c1:
        set_2_button = st.button('Start Installation (SET 2)', disabled=not set_2, help = "After running SET-1, retart your machine to run SET-2", on_click=SET_2_COMMANDS_MAIN)
    with c2c2:
        set_2_result = st.empty()
        if set_2_r == 0:
            set_2_result.empty()
        elif set_2_r == 1:
            set_2_result.success("SET 2 Commands ran successfully")
        elif set_2_r == -1:
            set_2_result.error("ERROR Running SET 2 Commands. Please Procees Manually")
        else:
            set_2_result.text("RUNNING SET 2. Please Wait")
            CLOSE_ALL()

    with c3c1:
        set_3_button = st.button('Start Installation (SET 3)', disabled=not set_3, help = "After running SET-2, retart your machine to run SET-3", on_click=SET_3_COMMANDS_MAIN)
    with c3c2:
        set_3_result = st.empty()
        if set_3_r == 0:
            set_3_result.empty()
        elif set_3_r == 1:
            set_3_result.success("SET 3 Commands ran successfully")
        elif set_3_r == -1:
            set_3_result.error("ERROR Running SET 3 Commands. Please Procees Manually")
        else:
            set_3_result.text("RUNNING SET 3. Please Wait")
            CLOSE_ALL()
        
    with c4c1:
        set_4_button = st.button('Start Installation (SET 4)', disabled=not set_4, help = "After running SET-3, retart your machine to run SET-4", on_click=SET_4_COMMANDS_MAIN)
    with c4c2:
        set_4_result = st.empty()
        if set_4_r == 0:
            set_4_result.empty()
        elif set_4_r == 1:
            set_4_result.success("SET 4 Commands ran successfully")
        elif set_4_r == -1:
            set_4_result.error("ERROR Running SET 4 Commands. Please Procees Manually")
        else:
            set_4_result.text("RUNNING SET 4. Please Wait")
            CLOSE_ALL()
            
    with c5c1:
        set_5_button = st.button('Start Installation (SET 5)', disabled=not set_5, help = "After running SET-4, retart your machine to run SET-5", on_click=SET_5_COMMANDS_MAIN)
    with c5c2:
        set_5_result = st.empty()
        if set_5_r == 0:
            set_5_result.empty()
        elif set_5_r == 1:
            set_5_result.success("SET 5 Commands ran successfully")
        elif set_5_r == -1:
            set_5_result.error("ERROR Running SET 5 Commands. Please Procees Manually")
        else:
            set_5_result.text("RUNNING SET 5. Please Wait")
            CLOSE_ALL()

    with c6c1:
        set_6_button = st.button('Start Installation (SET 6)', disabled=not set_6, help = "After running SET-5, retart your machine to run SET-6", on_click=SET_6_COMMANDS_MAIN)
    with c6c2:
        set_6_result = st.empty()
        if set_6_r == 0:
            set_6_result.empty()
        elif set_6_r == 1:
            set_6_result.success("SET 6 Commands ran successfully")
        elif set_6_r == -1:
            set_6_result.error("ERROR Running SET 6 Commands. Please Procees Manually")
        else:
            set_6_result.text("RUNNING SET 6. Please Wait")
            CLOSE_ALL()
        
    if set_1_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty()           

    if set_2_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty()    
                
    if set_3_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty()    

    if set_4_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty() 
            
    if set_5_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty()    

    if set_6_button:
        if username == '' or password == '':
            main_area_4.error('Please enter your username and password first')
        else:       
            main_area_4.empty()     

if __name__ == '__main__':  
    #setSV()
    getSV() 
    main()      
