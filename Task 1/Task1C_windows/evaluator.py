
'''
*****************************************************************************************
*
*        =================================================
*             Balancing Builder Bot (BB) Theme (eYRC 2023-24)
*        =================================================
*
*  This script is intended to check the versions of the installed
*  software/libraries in Task 1c of Balancing Builder Bot  (BB) Theme (eYRC 2023-24).
*
*  Filename:			task1c.py
*  Created:			10/10/2023
*  Last Modified:		16/10/2023
*  Author:			Isha, Jerish, e-Yantra Team
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
*
*****************************************************************************************
'''

# Import modules
import time
import signal
import string
import random
import base64
from datetime import datetime
import os, sys
import platform
sys.path.insert(0, 'pythonAPI')
from zmqRemoteApi import RemoteAPIClient
###############
from oauth2client.service_account import ServiceAccountCredentials
import gspread
import math
import random
import sys

os.system("pip install cryptocode")

import cryptocode

sim = None


scope = ['https://www.googleapis.com/auth/spreadsheets', "https://www.googleapis.com/auth/drive.file", "https://www.googleapis.com/auth/drive"]
credentials = {
		"type": "service_account",
		"project_id": "bb-logs-431613",
		"private_key_id": "a4506d44024339472c6cae03bddb65257200038b",
		"private_key": "-----BEGIN PRIVATE KEY-----\nMIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDA2GeVj68LVk+O\nsEDyOBAIhRH/0BFaD/rf2Swws+5G3EwC6qddOThCkMNsT4CuzpCLUeC9Na+bZCqZ\ngqv3+ygHCNFzCbVhtAu0/L5BlaYAtSOlwdayEJRcA4d1zyIZ9cvzDne8DKiMzcnU\nbQ1UsDCn5gET0GiHPmIYf7UJHc91TKRPa+echb+UPonL0q/BTXnpiNDDKBDPNdM3\nSszl1Hx0oP+SWiWqsaPdKTUjJzRSBm8Sh3TJrvfENehDwoAg6O+UXbTvy7Jx/EGU\nvmbss0zTD3239KihiYa1flQdAYWZv5E8d9U2YSrye+r8+YLbOMqXaPx/xzXPlrfK\n3wp0YzjBAgMBAAECggEABVwGTfNpMpwWmDE5DtpExM3Y9RZ14v5Cc0XsetqHTuAY\nuzSFC/vW/s+Z5MjVW1ZZkUuoXq3PKHcJUYyehaTs5PwnYQZ2LGXV9PTYv0ceztJ5\n8AWB918rVl7RPRKRgK3yefnvFTJ3XTlraGxS9GV0prfnVAME4qxWytZCxGL4FOsM\n40sS6+V5w10KkS0hyBJU9I6AjZMGcVI96hLQtMlFQOQV62EntrV5o0++Bs6lGczp\nrFt/0u+hazMUlDzY3iKvpgayreO5X7qoJJm6dDWSo6Sfq4wlKA3VveUG+cM3t72R\nRE9eljYOCfw6QD9ORa8XBZtNfN9aGXZIoqKbU92TaQKBgQDlNn30gmczKdwkSRQo\nCt/pPU3+CZWhu5rCnXfeHbfzU9u5GrWH40llgQDjgWgxZqtu/uWZyPuo6AtVEwUy\nkh6HqdzdDx42rhG1GrTbAIiSOn606FG8kQjFps8yqIfVWlL9awiTnCppy2O5QFU0\n9UDDzJubFWkdhR5b4TFft11lKQKBgQDXYeHHduYcjtCrlL4CTI7WWaAqA+VknAES\nxS/+CPZjMOv0OyjYnppI03gEiA2y70n/P4NmdUcwvDnJK7mdvuwrRcvsKF+UeV2y\ng0v3xUzaFaxyasfFZuRCYv7pr1cG/hRcmRI5fd9dlS4xWpkXqe/Yc8oU8pGKkbY9\nz4lf09LR2QKBgQCIUgVNIzU/X5j216OuQPF0ZSp6eLbOTqY3MrH0nxYlGG2oRDNM\nkye2v6eIpxERuG8i/2QMN1U82mzK9xnzPqX7p1GdA33DpXkQjcacLVAML8/lxfm+\nvT9LVe8KwOKwSBztbPfX2lv7OaSgq5tBeM9A4/JzpKM0lFQ+7sqPk51vKQKBgGV8\nGYaC36pVIL24OE+dAzC8ylsBuvTNDTRq9VIdpvrV8lgCCB0JnmjyO3rnII1Pcu5y\nXtfIKuMrzY6cq7lIXL+HA68i1uZ+yUdz1jfJH40i6T6AUeERujwNqU8y7y68SZvY\nBF5SkQznXfyjU79yszCqXm3AXhOM1PIK+A/PH2cBAoGAMLztRveOQxy0eU0Cl9KE\nvL4qbH2XYGn1aEHPTY8Dpx4A1varNKhXzUtHGPhVWPB1Qq//jot6+DgClQ6Kda4C\n+Vh8+1GUxv3DihmGcCvwb8BXwpaPhB9qq/2HnoWvq1bSKnRvA5ab8LC1z/DV1DRZ\nfXKQhr8q5KB9QfMmD8IzhzI=\n-----END PRIVATE KEY-----\n",
		"client_email": "balancing-builder-bot@bb-logs-431613.iam.gserviceaccount.com",
		"client_id": "105579116913873875207",
		"auth_uri": "https://accounts.google.com/o/oauth2/auth",
		"token_uri": "https://oauth2.googleapis.com/token",
		"auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
		"client_x509_cert_url": "https://www.googleapis.com/robot/v1/metadata/x509/balancing-builder-bot%40bb-logs-431613.iam.gserviceaccount.com",
		"universe_domain": "googleapis.com"
}

client = gspread.service_account_from_dict(credentials)
try:
    python_cell_update = client.open("BB_Logs").worksheet("task1C")
except:
    comment += "\n\t[ERROR] System not able to access internet!\n"
    print(comment)
    sys.exit()

Team_ID_c1 = python_cell_update.col_values(1)
Conda_env_c2 = python_cell_update.col_values(2)
Platform_name_c3 =  python_cell_update.col_values(3)
Current_time_c4 = python_cell_update.col_values(4)
check1_time_c5 = python_cell_update.col_values(5)
check2_time_c6 = python_cell_update.col_values(6)
Score_c7 = python_cell_update.col_values(7)		
Comment_c8	= python_cell_update.col_values(8)



# Flags to determine which modules are installed or not
platform_uname = None
conda_flag = None
python_flag = None


# Flags to determine whether the modules installed have correct version or not
conda_env_name_flag = None

# Flag and Global variables to determine whether the CoppeliaSim Remote API Server works fine
coppeliasim_remote_api_flag = None
setteling_time=None
clientID = 0


# Output file name
file_name = "task1c_output.txt"
fail_flag = False
comment = ""
yaw_setpoint = 0
count = 0
total_marks = 0.0
check1_time = 0
check2_time = 0

def update_sheet():
    global team_id, conda_env_name, platform_uname, current_time, comment, total_marks, check1_time, check2_time
    python_cell_update.update_cell(len(Team_ID_c1)+1,1,str(team_id)) 
    python_cell_update.update_cell(len(Conda_env_c2)+1,2,str(conda_env_name))
    python_cell_update.update_cell(len(Platform_name_c3)+1,3,str(platform_uname))
    python_cell_update.update_cell(len(Current_time_c4)+1,4,str(current_time))
    python_cell_update.update_cell(len(check1_time_c5)+1,5,str(check1_time))
    python_cell_update.update_cell(len(check2_time_c6)+1,6,str(check2_time))
    python_cell_update.update_cell(len(Score_c7)+1,7,str(total_marks))
    python_cell_update.update_cell(len(Comment_c8)+1,8,str(comment))


################# FUNCTION DEFINITIONS ##############
def objects_and_script_check(given_object):
    global sim

    given_object_has_script = -1
    total_objects = 0
    while(sim.getObjects(total_objects,sim.handle_all) != -1):
        total_objects = total_objects + 1
        # Total objects = last_object_id +1 (since starts from '0')

    total_scripts = 0
    for some_object in range(total_objects):

        some_script = sim.getScript(sim.scripttype_childscript,some_object)
        if(some_script != -1):
            total_scripts = total_scripts + 1

        if(some_object == given_object):
            if(some_script != -1):
                given_object_has_script = some_script

    return total_objects,total_scripts,given_object_has_script


def handler(signum, frame):
    global total_marks, task_flag, comment, platform_uname, local_grader_version, team_id, conda_env_name, check_flag1, check_flag2, current_time, child_script_text
    res = input("Ctrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        comment += "None"
        
        print("\n\t Following progress recorded: ")
        print("\n\t Score = " + str(total_marks))
        print("\n\t comment = " + comment)
        update_sheet()
        print("\n\t ###### WARNING : The score you get is just the local performance evaluation - to help improve the solution.")
        print("\t ###### WARNING : There will be additional checks performed on your solution on e-Yantra PORTAL after you submit.")
        print("\t ###### WARNING : Follow Task Rules Strictly.")
        print("\t ###### WARNING : e-Yantra Reserves the right to change the status of submission to FAILED any-time-after-submission... if any violation encountered")
        print("######################################################################################################################.........\n")
        task_flag=1 #for encrypted txt generation

        sim.stopSimulation()
        sys.exit()

        if os.path.exists(file_name):
            os.remove(file_name)


        if (task_flag == 1):
            # Create a 'output_file'
            output_file = open(file_name, "w")

            # Get the OS name on which this code is running
            platform_uname = cryptocode.encrypt(str(platform_uname), "ishajerishsahil")
            local_grader_version = cryptocode.encrypt("1.0", "sahilishajerish")
            team_id = cryptocode.encrypt(str(team_id), "jerishsahilisha")
            #------------Different encryption Key--------
            conda_env_name = cryptocode.encrypt(str(conda_env_name), "ishajerishsahil")
            total_marks = cryptocode.encrypt(str(total_marks), "jerishsahilisha")
            #--------------------------------------------
            current_time = datetime.now().strftime('%d-%m-%Y %H:%M:%S')
            current_time = cryptocode.encrypt(str(current_time), "sahilishajerish")
            child_script_text = cryptocode.encrypt(str(child_script_text), "jerishsahilisha")
            finaldata=[str(platform_uname)+"\n",str(team_id)+"\n",str(conda_env_name)+"\n",str(total_marks)+"\n",str(current_time)+"\n",str(child_script_text)+"\n"]
            output_file.writelines(finaldata)
            output_file.close()

    


################### TASK SPECIFC EVALUATION FUNCTION #############################
def test_task_1c():
    global  comment,count, yaw_setpoint, fail_flag, sim, platform_uname, total_marks, task_flag, platform_uname, check1_time, check2_time
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
    sim.setInt32Param(sim.intparam_idle_fps, 0)
    

    ################Looking for certain objects#################

    try:
        body        =sim.getObject('/body')
        right_joint =sim.getObject('/body/right_joint')
        right_wheel =sim.getObject('/body/right_joint/right_wheel')
        left_joint  =sim.getObject('/body/left_joint')
        left_wheel  =sim.getObject('/body/left_joint/left_wheel')

    except:
        print("Error: Check Name & Connections/path in Hierarchy !! Open Task_1C.ttt file in CoppeliaSim.")
        comment += "Check Name & Connections/path in Hierarchy !! Open Task_1C.ttt file in CoppeliaSim"
        update_sheet()
        sys.exit()


    try:
        Script=sim.getObject('/body/Script')
    except:
        comment += "Error: Script is missing or misplaced from its original location"
        print(comment)
        update_sheet()
        sys.exit()
    ########################################################

    ############Checking object properties###############

    if(sim.getObjectInt32Param(body,3003)!=0 or sim.getObjectInt32Param(right_wheel,3003)!=0 or sim.getObjectInt32Param(left_wheel,3003)!=0):
        comment = comment +  "Either of Bot Chassis or Wheels is not dynamically enabled. Do not change the scene."
        update_sheet()
        fail_flag = True


    if(sim.getJointMode(right_joint)[0]!=sim.jointmode_dynamic or sim.getJointMode(left_joint)[0]!=sim.jointmode_dynamic):
        comment = comment +  "Joints must be in dynamic mode"
        update_sheet()
        fail_flag = True

    if(sim.getObjectInt32Param(right_joint,sim.jointintparam_dynctrlmode)!=sim.jointdynctrl_velocity or sim.getObjectInt32Param(left_joint,sim.jointintparam_dynctrlmode)!=sim.jointdynctrl_velocity):
        comment = comment +  "Some Joints have wrong control mode."
        update_sheet()
        fail_flag = True

    if(sim.getObjectFloatParam(body,sim.shapefloatparam_mass)!=0.248 or sim.getObjectFloatParam(right_wheel,sim.shapefloatparam_mass)!=0.018 or sim.getObjectFloatParam(left_wheel,sim.shapefloatparam_mass)!=0.018):
        comment = comment +  "Object masses should not be changed."
        update_sheet()
        fail_flag = True

    if(sim.getSimulationTimeStep() != 0.01):
        comment = comment +  "Simulation Time Step should be 10ms only."
        update_sheet()
        fail_flag = True

    ################ Getting child Script ############################################
    given_object=sim.getObject('/body/Script')

    given_object_has_script = -1
    total_objects = 0
    while(sim.getObjects(total_objects, sim.handle_all) != -1):
        total_objects = total_objects + 1
        # Total objects = last_object_id +1 (since starts from '0')

    total_scripts = 0
    for some_object in range(total_objects):

        some_script = sim.getScript(sim.scripttype_childscript,some_object)
        if(some_script != -1):
            total_scripts = total_scripts + 1

        if(some_object == given_object):
            if(some_script != -1):
                given_object_has_script = some_script

    if(total_objects != 25):
        comment = comment + "Hierarchy damage. Some objects missing or added."
        fail_flag = True
    if(total_scripts != 2):
        comment = comment + "There should be ONE & only ONE script."
        fail_flag = True
    if(given_object_has_script < 24):
        comment = comment + "Script missing or misplaced"
        fail_flag = True

    child_script_text = sim.getScriptStringParam(given_object_has_script,sim.scriptstringparam_text)
    if fail_flag == True:
        print(comment)
        update_sheet()
        sys.exit()

    #####################################################

    simTime=0
    task_flag=0

    try:
        sim.setInt32Parameter(sim.intparam_dynamic_engine,sim.physics_bullet)
        ref_init_pose = sim.getObjectOrientation(body,-1)
        sim.startSimulation()

###################Freezing UI features#######################
        sim.setBoolParam(sim.boolparam_realtime_simulation, True)
        sim.setBoolParam(sim.boolparam_hierarchy_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_hierarchy_visible,False)
        sim.setBoolParam(sim.boolparam_browser_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_browser_visible,False)
        sim.setBoolParam(sim.boolparam_pause_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_stop_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_objectshift_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_objectrotate_toolbarbutton_enabled,False)
        sim.setBoolParam(sim.boolparam_statustext_open,False)
        sim.setBoolParam(sim.boolparam_infotext_visible,False)
##############################################################

    except:
        comment += "Error starting simulation."
        print(comment)
        update_sheet()
        sys.exit()


    simTime=0
    task_flag=0
    collide_flag = 0
    check_flag1 = 0
    check_flag2 = 0
    
    i = 0
    j = 0
    pos = 0
    check1_marks = 0
    check2_marks = 0

    signal.signal(signal.SIGINT, handler)
    try:
        r1 = sim.getObjectPosition(body,-1)
        start_end_point = sim.createPrimitiveShape(sim.primitiveshape_disc,[0.2,0.2,0.001])
        sim.setObjectPosition(start_end_point,-1,[0,0,0.001])
        sim.setObjectColor(start_end_point,0,sim.colorcomponent_ambient_diffuse,[1,1,0.5]) #Yellow
        sim.setObjectInt32Param(start_end_point,3003,1) #STATIC

        checkpoint1 = sim.createPrimitiveShape(sim.primitiveshape_spheroid,[0.15,0.15,0.15])#White
        sim.setObjectPosition(checkpoint1,-1,[0,0.5,0])

        checkpoint2 = sim.createPrimitiveShape(sim.primitiveshape_spheroid,[0.15,0.15,0.15])
        sim.setObjectPosition(checkpoint2,-1,[-0.5,0.75,0])
        sim.setObjectColor(checkpoint2,0,sim.colorcomponent_emission,[1,1,1]) #White

        #-----------------------------------------------------
        sim.setObjectColor(checkpoint1,0,sim.colorcomponent_emission,[1,1,1]) #White
        while simTime<=160:
            simTime=sim.getSimulationTime()
            body = sim.getObject('/body')
            floor = sim.getObject('/Floor')
            yaw_control = sim.getObjectOrientation(body, floor)
            yaw,pitch,roll = sim.alphaBetaGammaToYawPitchRoll(yaw_control[0],yaw_control[1],yaw_control[2])

            r2 = sim.getObjectPosition(body,-1)

            result, colliding_pair = sim.checkCollision(body,floor)
            if(result>0):
                collide_flag=1
                total_marks = 0.0
                break

            if check_flag1 == 0 or pos == 0.5:
                r3 = sim.getObjectPosition(checkpoint1,floor)
                pos = r3[1]
                if r3[1] == 0.5:
                    result1,distdata1,_=sim.checkDistance(body,checkpoint1,0.10)
                    if result1 == 1:
                        if distdata1[6] <=0.03 and collide_flag != 2:
                            collide_flag = 2
                            check_flag1 = 0
                            check1_marks = 0
                            total_marks = check1_marks
                            sim.setObjectColor(checkpoint1,0,sim.colorcomponent_emission,[1,0,0]) #red
                            time.sleep(2)
                            sim.removeObject(checkpoint1)

                        elif distdata1[6] <= 0.10 and distdata1[6] >=0.05 and collide_flag == 0 and float(abs(pitch)) <= 0.0436332:
                            i+=1
                            if i == 1:
                                check1_time = simTime
                                check1_marks = (((60-check1_time)/60)*20)
                                total_marks = check1_marks
                            check_flag1 = 1 #+/-2.5 degree
                            sim.setObjectColor(checkpoint1,0,sim.colorcomponent_emission,[0,1,0]) #green
                            if i == 50:
                                sim.removeObject(checkpoint1)
                                pos = 0

            else:
                result2,distdata2,_=sim.checkDistance(body,checkpoint2,0.10)
                if result2 == 1:
                    if distdata2[6] <=0.02 and collide_flag != 3:
                        collide_flag = 3
                        check_flag2 = 0
                        check2_marks = 0
                        total_marks += check2_marks
                        sim.setObjectColor(checkpoint2,0,sim.colorcomponent_emission,[1,0,0]) #red
                        time.sleep(2)
                        sim.removeObject(checkpoint2)
                        break


                    elif distdata2[6] <= 0.10 and distdata2[6] >=0.05 and collide_flag == 0 and float(abs(pitch)) <= 0.0436332:
                        j+=1
                        if j == 1:
                            check2_time = simTime
                            check2_marks = (((100-(check2_time-check1_time))/100)*30)
                            total_marks += check2_marks
                        check_flag2 = 1 #+/-2.5 degree
                        sim.setObjectColor(checkpoint2,0,sim.colorcomponent_emission,[0,1,0]) #green
                        if j == 50:
                            sim.removeObject(checkpoint2)
                            break


        #-----------------------------------------------------

        sim.removeObject(start_end_point)
        total_objects,total_scripts,given_object_has_script = objects_and_script_check(Script)

        # if(total_objects != 25):
        #     comment = comment + "Hierarchy damage. Some objects missing or added."
        #     update_sheet()
        #     fail_flag = True
        # if(total_scripts != 2):
        #     comment = comment + "There should be ONE & only ONE script."
        #     update_sheet()
        #     fail_flag = True
        # if(given_object_has_script == -1):
        #     comment = comment + "No script found under Script object"
        #     update_sheet()
        #     fail_flag = True

        child_script_text = sim.getScriptStringParam(given_object_has_script,sim.scriptstringparam_text)
      


    except Exception as error:
        comment = "Something went wrong during the simulation!"
        update_sheet()
        sim.removeObject(checkpoint1)
        sim.removeObject(checkpoint2)
        sim.removeObject(start_end_point)
        print("Score= ")
        print(total_marks)
        print(comment)
        update_sheet()
        sim.stopSimulation()
        sim.setBoolParam(sim.boolparam_realtime_simulation, True)
        sim.setBoolParam(sim.boolparam_hierarchy_toolbarbutton_enabled,True)
        sim.setBoolParam(sim.boolparam_hierarchy_visible,True)
        sim.setBoolParam(sim.boolparam_browser_toolbarbutton_enabled,True)
        sim.setBoolParam(sim.boolparam_browser_visible,True)
        sim.setBoolParam(sim.boolparam_pause_toolbarbutton_enabled,True)
        sim.setBoolParam(sim.boolparam_objectshift_toolbarbutton_enabled,True)
        sim.setBoolParam(sim.boolparam_objectrotate_toolbarbutton_enabled,True)
        sim.setBoolParam(sim.boolparam_statustext_open,True)
        sim.setBoolParam(sim.boolparam_infotext_visible,True)
        sim.setBoolParam(sim.boolparam_stop_toolbarbutton_enabled,True)
        sys.exit()

    sim.stopSimulation()
    ###################Enabling UI features#######################
    sim.setBoolParam(sim.boolparam_realtime_simulation, True)
    sim.setBoolParam(sim.boolparam_hierarchy_toolbarbutton_enabled,True)
    sim.setBoolParam(sim.boolparam_hierarchy_visible,True)
    sim.setBoolParam(sim.boolparam_browser_toolbarbutton_enabled,True)
    sim.setBoolParam(sim.boolparam_browser_visible,True)
    sim.setBoolParam(sim.boolparam_pause_toolbarbutton_enabled,True)
    sim.setBoolParam(sim.boolparam_objectshift_toolbarbutton_enabled,True)
    sim.setBoolParam(sim.boolparam_objectrotate_toolbarbutton_enabled,True)
    sim.setBoolParam(sim.boolparam_statustext_open,True)
    sim.setBoolParam(sim.boolparam_infotext_visible,True)
    sim.setBoolParam(sim.boolparam_stop_toolbarbutton_enabled,True)
    ##############################################################


    print("\n\t Following progress recorded: ")
    print("\n\t Score = " + str(total_marks))
    if check_flag1 == 1 and check_flag2 == 1 and collide_flag == 0:
        comment = "Good Work!! You crossed both the checkpoints!"
        # print(comment)

    if collide_flag == 0:
        if check_flag1 == 1 and check_flag2 == 0:
            comment = "Checkpoint1 crossed! Try for checkpoint2."

        if check_flag1 == 0 and check_flag2 == 1:
            comment = "Checkpoint2 crossed! Try for checkpoint1."
        
    elif collide_flag == 1:
        total_marks = 0.0
        comment = "Bot Collided with the floor!! Balance the bot first."
        
    print(comment) 
    print(check1_time)
    print(check2_time)
    update_sheet()

    # python_cell_update.update_cell(len(values_list_c)+1,1,str(team_id)+", "+"Task 1c, "+conda_env_name+", "+ "\n\t Score = " + str(total_marks)+",  "+platform_uname+",  "+current_time)
    print("\n\t ###### WARNING : The score you get is just the local performance evaluation - to help improve the solution.")
    print("\t ###### WARNING : There will be additional checks performed on your solution on e-Yantra PORTAL after you submit.")
    print("\t ###### WARNING : Follow Task Rules Strictly.")
    print("\t ###### WARNING : e-Yantra Reserves the right to change the status of submission to FAILED any-time-after-submission... if any violation encountered")
    task_flag=1 #for encrypted txt generation
    return total_marks,task_flag,child_script_text


############ Test the Setup function ##############
def test_setup():
    global team_id,conda_env_name,current_time,platform_uname, comment
    current_time = datetime.now().strftime('%d-%m-%Y %H:%M:%S')
    platform_uname = platform.uname().system
    try:
        team_id = int(input('\n\tEnter your Team ID (for e.g.: "1234" or "321"): '))
    except ValueError:
        comment += "\n\t[ERROR] Enter your Team ID which is an integer!\n"
        print(comment)
        update_sheet()
        sys.exit()
    # Get the current Conda environment name
    conda_env_name = os.environ['CONDA_DEFAULT_ENV']
    expected_conda_env_name = 'BB_' + str(team_id)

    # Check current Conda environment name is as expected
    if conda_env_name == expected_conda_env_name:
        conda_env_name_flag = 1
    else:
        conda_env_name_flag = 0
        comment += "\n\t[WARNING] Conda environment name is not found as expected, Make sure it is: BB_<team_id>, re-check the instructions!!"
        print("\n\t[WARNING] Conda environment name is not found as expected, Make sure it is: BB_%s, re-check the instructions\n" %(str(team_id)))
        update_sheet()
        sys.exit()


    ############### TASK SPECIFIC EVALUATION FUNCTION ######################
    total_marks,task_flag,child_script_text = test_task_1c()
    ########################################################################

    if os.path.exists(file_name):
        os.remove(file_name)

    if (task_flag == 1):
        # Create a 'output_file'
        output_file = open(file_name, "w")

        # Get the OS name on which this code is running
        platform_uname_en = cryptocode.encrypt(str(platform_uname), "ishajerishsahil")
        local_grader_version_en = cryptocode.encrypt("1.0", "sahilishajerish")
        team_id_en = cryptocode.encrypt(str(team_id), "jerishsahilisha")
        #------------Different encryption Key--------
        conda_env_name_en = cryptocode.encrypt(str(conda_env_name), "ishajerishsahil")
        total_marks_en = cryptocode.encrypt(str(total_marks), "jerishsahilisha")
        #--------------------------------------------
        current_time = datetime.now().strftime('%d-%m-%Y %H:%M:%S')
        current_time_en = cryptocode.encrypt(str(current_time), "sahilishajerish")
        child_script_text_en = cryptocode.encrypt(str(child_script_text), "jerishsahilisha")
        finaldata=[str(platform_uname_en)+"\n",str(team_id_en)+"\n",str(conda_env_name_en)+"\n",str(total_marks_en)+"\n",str(current_time_en)+"\n",str(child_script_text_en)+"\n"]
        output_file.writelines(finaldata)
        output_file.close()

        print("\t+--------------------------------------------------------------------------+")
        print("	|              $          $$$$$$$$$$$$$$$$$$$$$$$$         #              |")
        print("	|             $ $      $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$     # #             |")
        print("	|            $   $   $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  #   #            |")
        print("	|           $     $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$     #           |")
        print("	|           $     $$$$$$$$$####$$$$$$$$$$$$$####$$$$$$$$$$    #           |")
        print("	|            $   $$$$$$$$$######$$$$$$$$$$$######$$$$$$$$$$  #            |")
        print("	|             $ $$$$$$$$$$ #### $$$$$$$$$$$ #### $$$$$$$$$$$#             |")
        print("	|              $$$$$$$$$$$$    $$$$$$$$$$$$$    $$$$$$$$$$$$$             |")
        print("	|             $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$            |")
        print("	|            $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$           |")
        print("	|            $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$           |")
        print("	|            $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$           |")
        print("	|            $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$            |")
        print("	|            $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$            |")
        print("	|             $$$$$$  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  $$$$$$$            |")
        print("	|              $$$$$$   $$$$$$$$$$$$$$$$$$$$$$$$$$$$  $$$$$$$             |")
        print("	|               $$$$$$   $   $$$$$$$$$$$$$$$$$$   $  $$$$$$$              |")
        print("	|                $$$$$$$  $ $                  $ $ $$$$$$$                |")
        print("	|                 $$$$$$$$ $                    $ $$$$$$$                 |")
        print("	|                   $$$$$$$$$$$             $$$$$$$$$$$                   |")
        print("	|                      $$$$$$$$$$$$$$$$$$$$$$$$$$$$$                      |")
        print("	|                         $$$$$$$$$$$$$$$$$$$$$$$$                        |")
        print("	|                            $$$$$$$$$$$$$$$$$$                           |")
        print("\t+--------------------------------------------------------------------------+")


# Test the Setup and Check versions of the above imported modules
if __name__ == '__main__':


    print("\tBalancing Builder Bot Task-1c Local GRADER ")
    print("\t\t>>>>>>>>>>>>>")
    print("\t\t>>>>>>>>>>>>>")
    print("\t\t--VERSION-1--")
    print("\t\t>>>>>>>>>>>>>")
    print("\t\t>>>>>>>>>>>>>")

    test_setup()