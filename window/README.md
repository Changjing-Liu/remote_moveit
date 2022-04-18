### Introduction 

This repository contains Python classes to access ROS remotely from Windows/MacOS or Linux system using Python 2/3. 

### Requirements

- Python 2 or 3
- Roslibpy
- Scipy 
  
### Usage 

- Import the class *ROS_Robot* and create an instance of it and check the connection status: 

```python 
from ros_connect import Ros_Robot

# Connect to ROS
ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
# Returns True if connected, False if not.
connection_status = ros.is_connected()
# Disconnect to ROS
ros.disconnect()
```

- To get the joint position, speed and acceleration at different timetamps, or to get tool pose with respect to 'world' frame, you can do this

```python 
from ros_connect import Ros_Robot
import time
# Connect to ROS
ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
time.sleep(1) # Wait a second to get the connection set up properly

# Check connection status
if ros.is_connected():
    # Returns a dictionary object
    print (ros.get_joint_states())
    print (ros.get_current_pose())

# Disconnect to ROS
ros.diconnect()
```

You can also enable and disable the robot by calling 'enable_robot' and 'disable_robot' services. The reponse of the services is returned by the function call as follow: 

```python
from ros_connect import Ros_Robot
import time
# Connect to ROS
ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
time.sleep(1) # Wait a second to get the connection set up properly

# Check connection status
if ros.is_connected():
    # Returns a dictionary object like this:
    # response["message"] = String type
    # respnse["success"] = Boolean type
    response = ros.robot_enable()

    # Returns a dictionary object like this:
    # response["message"] = String type
    # respnse["success"] = Boolean type
    response = ros.robot_disable()

# Disconnect to ROS
ros.diconnect()
```

- You can also set and get the current speed scaling factor. Please remember that speed scaling is a factor greater than 0 and less than or equal to 1.0, which is multiplied by the maximum speed of each joint to get a target speed that will be considered as maximum speed during that session.  

```python
from ros_connect import Ros_Robot
import time
# Connect to ROS
ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
time.sleep(1) # Wait a second to get the connection set up properly

# Check connection status
if ros.is_connected():

    # Getting the current speed scale
    currentScale = ros.get_speed_scale()
    # Setting a speed scale of 0.5
    ros.set_speed_scale(scale=0.5)

# Disconnect to ROS
ros.diconnect()
```

   
- You can also send pose (Position and Orientation) with respect to any known frames as follow:


```python 
from ros_connect import Ros_Robot
from scipy.spatial.transform import Rotation 
import time

ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
time.sleep(1)

# Create rotation quaternion (x,y,z,w) 
r = Rotation.from_euler('zyx', [0,0,0], degrees=True).as_quat()

# Create positon vector (x,y,z)
p = [-0.3, 0.0, 0.0]

# Create a pose message as follow: 
pose = {
    "position" : {
        "x" : p[0],
        "y" : p[1],
        "z" : p[2]
    },
    "orientation" : {
        "x" : r[0],
        "y" : r[1],
        "z" : r[2],
        "w" : r[3]
    }
}

if ros.is_connected():
    # frame_id - The arm will move to the give pose with respect to this frame_id. 
    # plan_only - The arm will plan only, not move. To move the arm, set it to True
    # wait - If true, it will wait and add the next pose in queue otherwise it will cancel current pose target and move to the next one.
    ros.plan_path(pose, frame_id="link_6_t", plan_only=True)
    time.sleep(5)

ros.disconnect()
```

- You can also get the current *planned trajectory* from the *Ros_Robot* object as follow:


```python 
from ros_connect import Ros_Robot
from scipy.spatial.transform import Rotation 
import time

ros = Ros_Robot(ip_address='Your-Ubuntu-PC-IP', port='9090')
time.sleep(1)

# Create rotation quaternion (x,y,z,w) 
r = Rotation.from_euler('zyx', [0,0,0], degrees=True).as_quat()

# Create positon vector (x,y,z)
p = [-0.3, 0.0, 0.0]

# Create a pose message as follow: 
pose = {
    "position" : {
        "x" : p[0],
        "y" : p[1],
        "z" : p[2]
    },
    "orientation" : {
        "x" : r[0],
        "y" : r[1],
        "z" : r[2],
        "w" : r[3]
    }
}

if ros.is_connected():
    # frame_id - The arm will move to the give pose with respect to this frame_id. 
    # plan_only - The arm will plan only, not move. To move the arm, set it to True
    # wait - If true, it will wait and add the next pose in queue otherwise it will cancel current pose target and move to the next one.
    ros.plan_path(pose, frame_id="link_6_t", plan_only=True)
    time.sleep(2)

    # This will return a dictionary object as follow
    # -> plan_robot_path["start"] = A dictionary object with robot start state
    # -> plan_robot_path["trajectory"] = Another dictionary object with robot trajectory as points. Print them in console to get the better understanding.
    plan_robot_path = ros.get_planned_path()

ros.disconnect()
```

- Currently, these are the following frames in robot with which you can plan and execute a pose target:


  - base_link (Fixed)
  - world (Fixed, same as base_link)
  - base (Manufacturer frame, get_pose is returned in this frame)
  - link_1_s
  - link_2_l
  - link_3_u
  - link_4_r
  - link_5_b
  - link_6_t 

**Note:** Be very careful when you send execute command (*plan_only=False*) in **plan_path** function with a non-fixed frame id. As the non-fixed frame will be moving so the pose can end up in a different location than you imagine. 

For example, if we want to move robot arm tool to front of about 0.1 m, with respect to */link_6_t*, we should send orientation as (0,0,0,1.0) and position as (0, 0, 0.1). Now if we send the similar pose again, it will move the tool with respect to the current */link_6_t* pose, not the previous one.  


### Python Code Example

```python
from ros_connect import Ros_Robot
import time

ros = Ros_Robot(ip_address='192.168.129.4', port=9090)
time.sleep(1) 

success = False

while True:

    try:
        if ros.is_connected() and not success:
            response = ros.robot_enable()
            print (response)
            success = response['success']
        else:
            joint_states = ros.get_joint_states()
            tool_pose = ros.get_current_pose()
            speed_scalling_value = ros.get_speed_scale()

            print ("Joint States" , joint_states)
            print ("\n")
            print ("Tool Pose", tool_pose)
            print ("\n")
            print ("Speed Scaling Factor: ", speed_scalling_value)
            print ("\n")

            time.sleep(1)

    except KeyboardInterrupt:
        break

if ros.is_connected():
    ros.disconnect()

```


### Example 2  

``` Python
from ros_connect import Ros_Robot
import time
from scipy.spatial.transform import Rotation 

ros = Ros_Robot(ip_address='192.168.129.4', port=9090)
time.sleep(1) 
success = False
# Create rotation quaternion (x,y,z,w) 
r = Rotation.from_euler('zyx', [0,0,0], degrees=True).as_quat()
# Create positon vector (x,y,z)
p = [0.1, 0.0, 0.0]
# Create a pose message as follow: 
pose = { 
    "position" : { "x" : p[0], "y" : p[1], "z" : p[2] },
    "orientation" : {"x" : r[0], "y" : r[1], "z" : r[2], "w" : r[3]
    }
}
if ros.is_connected() and not success:
    response = ros.robot_enable()
    success = response['success']

if ros.is_connected() and success:
    # Get current joint states, tool pose
    joint_states = ros.get_joint_states()
    tool_pose = ros.get_current_pose()
    # Send desired pose for planning only
    ros.plan_path(pose, frame_id="link_6_t", plan_only=True)
    time.sleep(1)
    # Get planned path
    plannedPath = ros.get_planned_path()
    start = plannedPath['start']
    trajectory = plannedPath['trajectory']
    trajectoryPoints = trajectory['points']
    print ("\n Planned Path Start Position: ", start)
    print ("\n Number of points in planned trajectory: ", len(trajectoryPoints))
    
    for count, point in enumerate(trajectoryPoints):
        print ("\nPoint %d" %count, point)
        
if ros.is_connected():
    ros.disconnect()****

```

### Example 3 

``` python
from ros_connect import Ros_Robot
import time
from scipy.spatial.transform import Rotation 


def create_pose_msg( postion, rotation ):

# Create rotation quaternion (x,y,z,w) 
r = Rotation.from_euler('zyx', rotation , degrees=True).as_quat()
# Create positon vector (x,y,z)
p = postion
# Create a pose message as follow: 
pose = { 
    "position" : { "x" : p[0], "y" : p[1], "z" : p[2] },
    "orientation" : {"x" : r[0], "y" : r[1], "z" : r[2], "w" : r[3]
    }
}

ros = Ros_Robot(ip_address='192.168.129.4', port=9090)
time.sleep(1) 
success = False

if ros.is_connected() and not success:
    response = ros.robot_enable()
    success = response['success']

if ros.is_connected() and success:
    # Rotate arm to 
    ros.plan_path(pose, frame_id="link_6_t", plan_only=False)
    time.sleep(5)

    # Move tool backward 0.3 me
    pose['position']['x'] = -0.3
    ros.plan_path(pose, frame_id="link_6_t", plan_only=False)
    time.sleep(5)

    # Go back to home position
    ros.go_to_home()
    time.sleep(5)

while success:
    response = ros.robot_disable()
    success = response['success']

if ros.is_connected():
    ros.disconnect()
```


### Example 4 

``` python
from ros_connect import Ros_Robot
import time
from scipy.spatial.transform import Rotation 

def create_pose_msg( postion, rotation ):
    # Create rotation quaternion (x,y,z,w) 
    r = Rotation.from_euler('zyx', rotation , degrees=True).as_quat()
    # Create positon vector (x,y,z)
    p = postion
    # Create a pose message as follow: 
    pose = { 
        "position" : { "x" : p[0], "y" : p[1], "z" : p[2] },
        "orientation" : {"x" : r[0], "y" : r[1], "z" : r[2], "w" : r[3]
        }
    }
    return pose

ros = Ros_Robot(ip_address='192.168.129.4', port=9090)
time.sleep(1) 
success = False

if ros.is_connected() and not success:
    response = ros.robot_enable()
    success = response['success']

if ros.is_connected() and success:

    ros.set_speed_scale(0.1)
    rotation_sets = [ [90, 0, 0], [0, 90, 0], [0, 0, 90] ]
    position = [0, 0, 0]
    
    for rotation in rotation_sets: 
        # Rotations 
        pose = create_pose_msg( position, rotation )
        ros.plan_path(pose, frame_id="link_6_t", plan_only=False)
        time.sleep(5)

        # Go back to home position
        ros.go_to_home()
        time.sleep(5)

while success:
    response = ros.robot_disable()
    success = response['success']

if ros.is_connected():
    ros.disconnect()

```


### Edits : 

A '/base' frame is added to match the manufacturer's coordinate frames with that of ROS and get the same tool pose as that on Teach Pendent of the robotic arm. This frame is assigned according to manufacturer specification (using datasheets). 

Please make sure that when you get 'pose' from get_pose function, carefully notice if you are getting the frame_id as '/base' or 'base_link'. If it is 'base_link' then the coordinate frame description is in ROS specification but it is 'base' then the coordinate frame description is in manufacturer (Yaksawa) description. 
  
![Base frame visual in RViZ and TF](./images/base_frame_visual.PNG)