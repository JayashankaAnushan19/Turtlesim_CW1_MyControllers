# turtlesim_my_controllers_pde4430_cw1

**Coursework 01 for Mudule: PDE4430 Mobile Robotics 2024-2025**. Tasks to be complete as follows;
1. Teleoperation using the keyboard, with an option to change movement speed  
2. Autonomous navigation to any given coordinate in the Turtlesim window 
3. Avoiding wall collision – Override movement if wall hitting is imminent
4. Vacuum Cleaning behaviour – Covering the entire window in an efficient manner 
5. Multiple turtles vacuum cleaning behaviour 

**Implementation Language:** Python
**Python Version:** This project is intended for `Python 3.8.10`, as per the original author's specification.

## Task 01 - Teleoperation using the keyboard, with an option to change movement speed  

*Note:* This task involves modifying/recreate the `turtlesim_teleop_key` control script to include a speed control option and keyboard inputs for enhanced control of the turtlesim simulation.

1. **Objective:** Recreate the in-built turtlesim_teleop_key script with added functionalities:
    - Speed Control: Allow dynamic adjustment of the turtle's movement speed.   
    - Enhanced Keyboard Controls: Implement custom keyboard inputs for better control.
2. **Implementation Note** Developed the task to control the turtlesim bot with keyboard inputs of "WASD" or arrow kes and using "+" or "-", can be control the speed.

>> File : `turtlesim_myteleop_key.py`


## Task 02 - Autonomous navigation to any given coordinate in the Turtlesim window 

*Note:* This task indicating to develop a program to move the bot to given cordinates.

1. **Objective:** Bot should navigate the given cordinates from currect position.
2. **Implementation Note** This task has been developed to move the turtlesim bot to the given target from the currect position. Used Pythagorean theorem to calculate the distance and depend on the distance bot will change the speed (Added a constant value to change the speed). If the target is too far, bot will move faster and if the target is close, bot will move slowly to achive the target cordinates accuratly.

>> File : `turtlesim_achive_target.py`


## Task 03 - Avoiding wall collision – Override movement if wall hitting is imminent 

*Note:* This task indicating to develop a program to move the bot to given cordinates. But bot should not cross the defined boundries and alert the user as bot arrived to boundry.

1. **Objective:** Define the imaginary boundry, able to move the bot and Bot should stop when arrive the boundry and alert the user.
2. **Implementation Note** 

>> File : `turtlesim_travelWithBoundries.py`


## Task 04 - Vacuum Cleaning behaviour – Covering the entire window in an efficient manner 

## Task 05 - Multiple turtles vacuum cleaning behaviour

*Developer note: 2 is good, 3 or more is great* 

## Reference





