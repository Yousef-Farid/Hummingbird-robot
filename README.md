## Install
- Python 3.8.10
* MuJoCo 2.3.7
  
  
## Running the SoftHand Model

To load the standalone SoftHand model in MuJoCo, run:

```bash 
python tendon_finger_control.py
```


This will open the SoftHand simulation shown in the figure below.
By adjusting the Control Panel on the top-right of the MuJoCo interface, you can modify the tendon forces applied to each finger.
Changing these values allows you to observe the resulting finger motion and grasp behavior.

<img width="2460" height="757" alt="Softhand_mujoco" src="https://github.com/user-attachments/assets/807550bc-14a0-436a-ac47-d14e50c81f40" />





## Running the Franka–SoftHand Manipulation Demo

To load the full Franka Emika Panda + SoftHand model for pick-and-place or object manipulation tasks, run: 

```bash 
python franka_softhand_tendon_control.py
```
 

This simulation (shown below) integrates the SoftHand with the Franka arm. The hand is controlled through tendon force commands, enabling compliant grasping and interaction with objects. 

<img width="2740" height="760" alt="franka_softhand_mujoco" src="https://github.com/user-attachments/assets/52e787d1-01a1-4107-8ae8-4a8ed7a14a40" />
