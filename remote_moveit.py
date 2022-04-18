#! /usr/bin/env python
# coding=UTF-8

# Imports 
import sys
import copy
import rospy
import time
import sys
import copy
import rospy
import moveit_commander
from math import pi
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
#-----------------lcj_added-------------
from moveit_msgs.msg import RobotTrajectory
#-----------------end-------------------
import tf 

class RemoteMoveIt:

    # Initialize the RemoteMoveIt Class
    # Setup all the subscriber and publishers 
    def __init__(self, group_name="arm"):

        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',DisplayTrajectory,queue_size=20)
        #-------------------------------------added
        #发布者 轨迹规划成功率
        self.plan_fraction_publisher = rospy.Publisher('/remote_get_plan_fraction',Float32,queue_size=20)
        #----------------------------------------end

        rospy.Subscriber('/remote_named_goal_pose', String, self.update_named_goal_pose)
        rospy.Subscriber('/remote_goal_pose', PoseStamped, self.update_goal_pose)
        rospy.Subscriber('/remote_cartesian_plan', PoseArray, self.update_cartesian_plan)
        #-----------------------------------------lcj_added--------------------------------------------
        #订阅者 接收笛卡尔规划执行
        rospy.Subscriber('/remote_cartesian_execute', String, self.update_cartesian_execute)
        #订阅者 接收速度系数
        rospy.Subscriber('/remote_set_max_velocity_scaling_factor', Float32, self.update_max_velocity_scaling_factor)
        #订阅者 接收加速度系数
        rospy.Subscriber('/remote_set_max_acceleration_scaling_factor', Float32, self.update_max_acceleration_scaling_factor)
        #-----------------------------------------end--------------------------------------------
        rospy.Subscriber('/remote_cartesian_plan_execute', PoseArray, self.update_cartesian_plan_execute)

        rospy.Subscriber('/joint_states', JointState, self.update_joint_states)
        self.pub_fk = rospy.Publisher('/remote_get_pose', PoseStamped, queue_size=20)

        #-------------lcj_add------------------
        self.plan=RobotTrajectory() #类成员plan 规划轨迹
        self.step=Float32()         #类成员step 笛卡尔规划补偿
        self.velocity_scaling_factor=Float32() #类成员velocity_scaling_factor 速度系数
        self.acceleration_scaling_factor=Float32() #类成员acceleration_scaling_factor 加速度系数
        #---------------end--------------------
        self.current_joint_states = JointState()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10)
        self.plannar_time = 0

    #-------------------------------lcj_added---------------------
    # 更新速度系数
    def update_max_velocity_scaling_factor(self,msg):
        rospy.logwarn("set the max_velocity_scaling_factor to %f",(msg.data))
        self.velocity_scaling_factor=msg.data #将接收消息保存在 类成员 速度系数 中
        self.move_group.set_max_velocity_scaling_factor(self.velocity_scaling_factor) #调用move_group设置第一类速度控制
    # 更新加速度系数
    def update_max_acceleration_scaling_factor(self,msg):
        #
        #
        #self.move_group.remember_joint_values("pose1")
        #self.move_group.set_named_target("pose1")
        #
        self.acceleration_scaling_factor=msg.data
        rospy.logwarn("set the max_acceleration_scaling_factor to %f",(msg.data))
        self.move_group.set_max_acceleration_scaling_factor(msg.data)
    #-------------------------------------------------------

    # Callback to named goal pose 
    def update_named_goal_pose(self, msg):
        self.move_group.clear_pose_targets()
        all_targets = self.move_group.get_named_targets()

        if msg.data in all_targets:
            self.move_group.set_named_target(msg.data)
            rospy.logwarn("Moving the robot to %s pose",str(msg.data) )
            self.move_group.go(wait=True)        
            
    # Callback to given goal pose
    def update_goal_pose(self, msg):
        rospy.logwarn("Moving the robot to goal pose")
        self.move_group.clear_pose_targets()

        # Setting current moveit state to joint state
        moveit_start_state = RobotState()
        moveit_start_state.joint_state = self.current_joint_states
        self.move_group.set_start_state(moveit_start_state)
        rospy.logwarn("Moveit start state is updated")

        frame_id , plan_only = msg.header.frame_id.split(',')
        plan_only = bool(int(plan_only))  
        msg.header.frame_id = frame_id  
        plan = self.move_group.plan(msg)
        if not plan_only:
            self.move_group.execute(plan, wait=True)

    # 
    def update_joint_states(self, msg):
        self.current_joint_states = msg

    # 
    def update_cartesian_plan(self, msg):
        self.move_group.set_start_state_to_current_state()
        # Set the waypoints 
        # Must be a list of poses
        waypoints = []
        for point in msg.poses:
            waypoints.append(copy.deepcopy(point))
        #---------------------------------lcj_added
        #(plan, fraction) = self.move_group.compute_cartesian_path(  waypoints, msg.step, 0.0)
        self.move_group.set_planning_time(10)
        #---------------------------------------end
        (plan, fraction) = self.move_group.compute_cartesian_path(  waypoints, 0.01, 0.0)
        #---------------------------------lcj_added----------------------------------
        #self.move_group.retime_trajectory(self.robot.get_current_state(), plan,  0.1,  0.1, algorithm="iterative_spline_parameterization")
        # 第二类速度控制，输入参数为 类成员 速度系数 加速度系数
        self.plan=self.move_group.retime_trajectory(self.robot.get_current_state(), plan,  self.velocity_scaling_factor, self.acceleration_scaling_factor, algorithm="iterative_time_parameterization")
        #--------------------------------------end-------------------------------------
        start =  JointState()
        start.header = self.plan.joint_trajectory.header
        start.name = self.plan.joint_trajectory.joint_names
        start.position = self.plan.joint_trajectory.points[0].positions
        start.velocity = self.plan.joint_trajectory.points[0].velocities
        start.effort = self.plan.joint_trajectory.points[0].effort

        display = DisplayTrajectory()
        display.trajectory.append( self.plan )
        display.trajectory_start.joint_state = start
        self.display_trajectory_publisher.publish(display)
        #------------------------------
        # 发布规划成功率
        self.plan_fraction_publisher.publish(fraction)
        #-------------------
    #-------------------------------------lcj_added----------------------------------------
    # 规划执行
    def update_cartesian_execute(self, msg):

        self.plan.joint_trajectory.points = self.remove_duplicates( self.plan.joint_trajectory.points) 
        self.move_group.execute(self.plan , wait=False)

    #---------------------------------------end---------------------------------------------

    #   
    def update_cartesian_plan_execute(self, msg):
        rospy.logwarn("Cartesian planning and execution request #%d" ,self.plannar_time )
        # try:
        self.plannar_time  +=1
        waypoints = []
        for point in msg.poses:
            waypoints.append(copy.deepcopy(point))

        (plan, fraction) = self.move_group.compute_cartesian_path(  waypoints, 0.01, 0.0 )

        # valid algorithms are 
        # 1) iterative_time_parameterization
        # 2) iterative_spline_parameterization #
        # 3) time_optimal_trajectory_generation
        #-------------------------
        #self.move_group.retime_trajectory(self.robot.get_current_state(), plan,  0.1,  0.1, algorithm="iterative_spline_parameterization")
        plan=self.move_group.retime_trajectory(self.robot.get_current_state(), plan,  0.3, 0.3, algorithm="iterative_time_parameterization")
        #----------------------------
     
        rospy.logwarn("Planning success percentage: %f", fraction)
        #
        plan.joint_trajectory.points = self.remove_duplicates( plan.joint_trajectory.points) 
        self.move_group.execute(plan , wait=False)

        # plan.joint_trajectory.points = self.remove_duplicates( plan.joint_trajectory.points) 
        # self.move_group.execute(plan , wait=False)
        # except: 
            # rospy.logerr("error in cartesian planning")


    def remove_duplicates(self, points):
        last_time_from_start = -1
        valid_points = []
        for point in points : 
            time_from_start = point.time_from_start.secs + point.time_from_start.nsecs * 1e-9
            if (time_from_start != last_time_from_start):
                valid_points.append(point)
                last_time_from_start = time_from_start
        
        return valid_points

    # Publishes current pose at 10 Hz
    def loop (self):
        while not rospy.is_shutdown():
            try:
                pose = self.move_group.get_current_pose()
                
                try : 
                    ref_frame = rospy.get_param("/remote_reference_frame")
                    tar_frame = rospy.get_param("/remote_target_frame")

                    self.listener.waitForTransform(ref_frame, tar_frame, rospy.Time(0), rospy.Duration(5.0))
                    (translation, rotation) = self.listener.lookupTransform(ref_frame, tar_frame, rospy.Time(0))
                    # rospy.loginfo(translation)
                    # rospy.loginfo(rotation)
                    pose.pose.position.x = translation[0]
                    pose.pose.position.y = translation[1]
                    pose.pose.position.z = translation[2]
                    
                    pose.pose.orientation.x = rotation[0]
                    pose.pose.orientation.y = rotation[1]
                    pose.pose.orientation.z = rotation[2]
                    pose.pose.orientation.w = rotation[3]
                    pose.header.frame_id = ref_frame

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue  

                self.pub_fk.publish( pose )
                self.rate.sleep()              
            except :
                rospy.logwarn("ROS shutdown is requested")
                continue
            
            


if __name__ == "__main__":

    rospy.init_node("remote_moveit_node")
    rm = RemoteMoveIt()
    rm.loop()
    rospy.spin()