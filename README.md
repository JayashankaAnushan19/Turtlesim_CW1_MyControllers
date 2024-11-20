# Turtlesim_My_Controllers_PDE4430_CW1


**Coursework 01 for Mudule: PDE4430 Mobile Robotics 2024-2025**. Tasks to be complete as follows;
1. Teleoperation using the keyboard, with an option to change movement speed  
2. Autonomous navigation to any given coordinate in the Turtlesim window 
3. Avoiding wall collision – Override movement if wall hitting is imminent
4. Vacuum Cleaning behaviour – Covering the entire window in an efficient manner 
5. Multiple turtles vacuum cleaning behaviour <br><br>


* **Implementation Language:** Python
* **Python Version:** This project is intended for `Python 3.8.10`, as per the original author's specification.
<br>

## 1). Task 01 - Teleoperation using the keyboard, with an option to change movement speed  

*Note:* This task involves modifying/recreate the `turtlesim_teleop_key` control script to include a speed control option and keyboard inputs for enhanced control of the turtlesim simulation.

1. **Objective:** Recreate the in-built turtlesim_teleop_key script with added functionalities:
    - Speed Control: Allow dynamic adjustment of the turtle's movement speed.   
    - Enhanced Keyboard Controls: Implement custom keyboard inputs for better control.
2. **Implementation Note:** Developed the task to control the turtlesim bot with keyboard inputs of "WASD" or arrow kes and using "+" or "-", can be control the speed. 

>>**Development Code :** `turtlesim_myteleop_key.py`
>>**Task Running file :** `runTask01.launch`

<br>

## 2).Task 02 - Autonomous navigation to any given coordinate in the Turtlesim window 

*Note:* This task indicating to develop a program to move the bot to given cordinates.

1. **Objective:** Bot should navigate the given cordinates from currect position.
2. **Implementation Note:** This task has been developed to move the turtlesim bot to the given target from the currect position. Used Pythagorean theorem to calculate the distance and depend on the distance bot will change the speed (Added a constant value to change the speed). If the target is too far, bot will move faster and if the target is close, bot will move slowly to achive the target cordinates accuratly.

>>**Development Code :** `turtlesim_achive_target.py`
>>**Task Running file :** `runTask02.launch`

<br>

## 3). Task 03 - Avoiding wall collision – Override movement if wall hitting is imminent 

*Note:* This task indicating to develop a program to move the bot to given cordinates. But bot should not cross the defined boundries and alert the user as bot arrived to boundry.

1. **Objective:** Define the imaginary boundry, able to move the bot and Bot should stop when arrive the boundry and alert the user.
2. **Implementation Note:** This task will take the arguments from the user to set the boundries first and later user can control the bot using arrow keys or 'AWSD' keys. Speed will control by '+' and '-' to increase or decrease.

>>**Development Code :** `turtlesim_travelWithBoundries.py`
>>**Task Running file :** `runTask03.launch`

<br>

## 4). Task 04 - Vacuum Cleaning behaviour – Covering the entire window in an efficient manner 

1. **Objective:** Entered turtlesim bot need to behave as a vaccum clearner. Need to conver entire area.
2. **Implementation Note:** This task have already defined values to identify the limits and bot will not cross the line. 

>>**Development Code :** `turtlesim_vacuumCleaningBehaviour.py`
>>**Task Running file :** `runTask04.launch`

<br>

## 5). Task 05 - Multiple turtles vacuum cleaning behaviour

1. **Objective:** Entered any number of turtlesim bot need to behave as a vaccum clearner. Space need to devide as per the number of bots and they have to do the job individually same time.
2. **Implementation Note:** This task have already defined values to identify the limits and bot will not cross the line. 

>>**Development Code :** `turtlesim_vacuumCleaningBehaviour.py`
>>**Task Running file :** `runTask05.launch`

<br>

## 6). Run the developed programs
All the programs combined to launch files and for execute the task, able to run the launch file using below steps.
1. First need to navigate the package using `roscd turtlesim_my_controllers_pde4430_cw1`
2. Then need to locate launch file folder using `cd launch`
3. Then as per the task which is need to execute, can be call by using `roslaunch <package name> <launch file>`
- **Example 01** : To run Task-o1; 
>>`roslaunch turtlesim_my_controllers_pde4430_cw1 runTask01.launch` 
- **Example 02** : To run Task-o2; 
>>`roslaunch turtlesim_my_controllers_pde4430_cw1 runTask02.launch` 

<br>

## Reference
http://wiki.ros.org/turtlesim
https://stackoverflow.com/questions/954834/how-do-i-use-raw-input-in-python-3
