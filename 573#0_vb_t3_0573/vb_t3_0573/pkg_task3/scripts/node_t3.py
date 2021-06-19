#! /usr/bin/env python
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import tf2_ros
import tf2_msgs.msg
import os
import math
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse
from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse
from pkg_vb_sim.msg import LogicalCameraImage
from gazebo_msgs.srv import ApplyBodyWrench, GetModelProperties, GetWorldProperties, SetModelState
from geometry_msgs.msg import Wrench
class CartesianPath:
    def __init__(self):
        rospy.init_node('node_eg5_waypoints', anonymous=True)
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        waypoints = []
        waypoints.append(self._group.get_current_pose().pose)
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self._group.compute_cartesian_path(waypoints,0.01,0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]
        self._group.execute(plan)
    def set_joint_angles(self, arg_list_joint_angles):
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan
    def go_to_pose(self, arg_pose):
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
        return flag_plan
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
def main():
    ur5 = CartesianPath()
    lst_joint_angles_pre =[math.radians(180),math.radians(-40),math.radians(30),math.radians(-80),math.radians(-90),math.radians(0)]                         
    lst_joint_angles_0 = [math.radians(0),math.radians(-20),math.radians(0),math.radians(-70),math.radians(-90),math.radians(0)]
    lst_joint_angles_1 = [math.radians(0),math.radians(-20),math.radians(0),math.radians(-70),math.radians(-90),math.radians(0)]
    lst_joint_angles_2 = [math.radians(70), math.radians(-20),math.radians(0),math.radians(-70),math.radians(-90),math.radians(0)]
    lst_joint_angles_3 = [math.radians(180),math.radians(-20),math.radians(0),math.radians(-70),math.radians(-90),math.radians(0)]
    lst_joint_angles_4 = [math.radians(-90),math.radians(-20),math.radians(0),math.radians(-70),math.radians(-90),math.radians(0)]
    arg_frame_1 = "logical_camera_2_packagen1_frame"
    arg_frame_3 = "logical_camera_2_packagen2_frame"
    arg_frame_4 = "logical_camera_2_packagen3_frame"
    arg_frame_2 = "ur5_wrist_3_link"
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    vacuumGripperService = '/eyrc/vb/ur5_1/activate_vacuum_gripper'
    conveyorService='/eyrc/vb/conveyor/set_power'
    rospy.wait_for_service(conveyorService)
    con = rospy.ServiceProxy(conveyorService,conveyorBeltPowerMsg) 
    rospy.wait_for_service(vacuumGripperService)
    vacuum_Gripper = rospy.ServiceProxy(vacuumGripperService,vacuumGripper)
    box_length = 0.15              
    vacuum_gripper_width = 0.115    
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5
    arr=["logical_camera_2_packagen1_frame","logical_camera_2_packagen2_frame","logical_camera_2_packagen3_frame"]
    fp=[lst_joint_angles_2,lst_joint_angles_0,lst_joint_angles_4]
    for i in range(3):
        ur5.go_to_pose(ur5_2_home_pose)
        resu = con(100)
        while not rospy.is_shutdown():           
            try:
                trans =tfBuffer.lookup_transform(arr[i], arg_frame_2, rospy.Time())
                x1=trans.transform.translation.x
                y1=trans.transform.translation.y
                z1=trans.transform.translation.z
                rospy.loginfo(  "\n" +"Translation: \n" +"x: {} \n".format(x1) +"y: {} \n".format(y1))   
                resu = con(35)
                if(y1>=0):
                   resu = con(0) 
                   rospy.sleep(1)
                   x1=trans.transform.translation.x
                   y1=trans.transform.translation.y
                   z1=trans.transform.translation.z  
                   break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("TF error")       
        ur5.ee_cartesian_translation(-x1,-y1,-z1+0.19)
        try:
            resp = vacuum_Gripper(activate_vacuum_gripper=True)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("\nVacuum gripper activated.\n")
        ur5.set_joint_angles(fp[i])
        resu=con(25)
        try:
            resp = vacuum_Gripper(activate_vacuum_gripper=False)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        print("\nVacuum gripper de-activated.\n")
        ur5.go_to_pose(ur5_2_home_pose)
    print("task Successfully done")       
    del ur5
if __name__ == '__main__':
    main()

