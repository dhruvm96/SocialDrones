# socialdrone
Crazyflie Research Honors Senior Design 
-------------------------------
		Team 9 - Social Drones
-------------------------------
		Kushantha Attanayake
		Manuel Navarro Catalán
		Dhruv Mathew
		Abhinav Mohan
		Trevor Murdock
		Akilan Pughazhendi
		Michael Smith

cfcli is a python script for easily controlling automatic flight of Crazyflie drone. 
Crazy Flie Command Line Interface supports various commands and runs of commands from the command line. 

3 other builds of cfcli
Cfcli_ex.py (ex = extended) uses cflib to connect and send movement commands to the Crazyflie. 
    Multiple input methods are supported, including command line, PS4 controller, 
    command file, and of course, computer vision. Invoke `python cfcli_ex.py -h` to see a list of options.
    
Cfcli_sim.py is a slimmed verision of cfcli_ex.py made to run on the raspberry pi zero and connect to AirSim over wifi.

Cfcli_cv.py is a slimmed version of cfcli_ex.py. It only contains the input method for computer vision. 
    Future goals of the project would be to get this to run on the raspberry pi by merging features with 
    cfcli_sim and adding methods to communicate with the drone. 

