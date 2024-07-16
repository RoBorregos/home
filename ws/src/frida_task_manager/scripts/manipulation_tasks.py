#!/usr/bin/env python3

"""
This script manages the implementation of each of the manipulation tasks
"""

### Import libraries
import rospy
import actionlib

import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


### ROS messages
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool
from frida_hri_interfaces.msg import Command, CommandList
from frida_manipulation_interfaces.msg import manipulationPickAndPlaceAction, manipulationPickAndPlaceGoal, manipulationPickAndPlaceResult, manipulationPickAndPlaceFeedback
from frida_manipulation_interfaces.msg import MoveJointAction, MoveJointGoal, MoveJointResult, MoveJointFeedback, moveXYZ
from frida_manipulation_interfaces.srv import Gripper, MovePose

MANIPULATION_SERVER = "/manipulationServer"
ARM_SERVER = "/arm_joints_as"
MOVE_SERVER = "/cartesian_movement_services/MovePose"
PLACE_TARGET = -5
POUR_TARGET = -10


# joints are -90, -70, -65, 0, 15, 45 in radians
OBSERVE_JOINT_POSITION = [-1.57, -1.22, -1.13, 0.0, 0.12, 0.78]
# PICK IS -90, -50, -105, -180, -60, 225
PICK_JOINT_POSITION = [-1.5707963705062866, -0.6108652353286743, -1.5707963705062866, 3.1415927410125732, -0.6108652353286743, -2.356194496154785]
RECEIVE_JOINT_POSITION = [-1.5707963705062866, -1.2217304706573486, -1.1344640254974365, 0.0, 0.7853981852531433, -0.78]
NAV_JOINT_POSITION = [-1.511849, -1.016862, -1.598619, 3.141800, -1.023188, -2.376529]
# Carrying angles -90, -70, -65, 0, 100, -135
CARRYING_JOINT_POSITION = [-1.3405917882919312, -0.9290976524353027, -1.1157159805297852, 0.04841051623225212, -0.11087805032730103, 0.6598629355430603]

# Positions
MANIPULATION_POSITIONS = {
    "NAV_JOINT_POSITION": NAV_JOINT_POSITION,
    "OBSERVE_JOINT_POSITION": OBSERVE_JOINT_POSITION,
    "PICK_JOINT_POSITION": PICK_JOINT_POSITION,
    "RECEIVE_JOINT_POSITION": RECEIVE_JOINT_POSITION,
    "CARRYING_JOINT_POSITION": CARRYING_JOINT_POSITION
    
}

class TasksManipulation:
    """Manager for the manipulation area tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["pick", "place", "grab", "give", "open", "close", "move_arm", "place_shelf"]

    OBJECTS_DICT = {
        "zucaritas": "cocacola",
        "cookies": "galletas",
        "cookie": "galletas",
        "snack": 5,
        "snacks": 5
    }

    def __init__(self, fake = False) -> None:
        
        self.FAKE_TASKS = fake
        if not self.FAKE_TASKS:
            rospy.loginfo("[INFO] Waiting for manipulation server")
            self.manipulation_client = actionlib.SimpleActionClient(MANIPULATION_SERVER, manipulationPickAndPlaceAction)
            self.move_arm_client = actionlib.SimpleActionClient(ARM_SERVER, MoveJointAction)
            self.gripper_service = rospy.ServiceProxy('/gripper_service', Gripper)     
            self.move_pose_client = rospy.ServiceProxy(MOVE_SERVER,MovePose)       
            rospy.loginfo("[INFO] Connecting to manipulation_server")
            if not self.manipulation_client.wait_for_server(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Manipulation server not initialized")
            rospy.loginfo("[INFO] Connecting to arm group")
            self.arm_group = moveit_commander.MoveGroupCommander("arm", wait_for_servers = 0)
            self.toggle_octomap = rospy.ServiceProxy('/toggle_octomap', SetBool)
            
            rospy.loginfo("[INFO] Connecting to arm_server")
            if not self.move_arm_client.wait_for_server(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Arm server not initialized")
            rospy.loginfo("[INFO] Connecting to gripper_service")
            if not self.gripper_service.wait_for_service(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Gripper service not initialized")
            rospy.loginfo("[INFO] Connecting to move_pose_client")
            if not self.move_pose_client.wait_for_service(timeout=rospy.Duration(10.0)):
                rospy.logerr("[SUCCESS] Move pose client not initialized")
            # rospy.loginfo("[INFO] Connecting to arm group")
                
            self.OBSERVER = [-1.5700864791870117, -1.1652400493621826, -1.4244275093078613, -6.2831220626831055, 0.3724396228790283, -5.487866401672363, 0.0] ## TO RADIANDS USING PI
            self.NAV_ARM = [-1.5701032876968384, -1.1651480197906494, -1.424232840538025, -6.283036231994629, 0.9078435301780701, -5.487793445587158, 0.0]

        rospy.loginfo("[SUCCESS] Manipulation Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] Manipulation Command")
        if command == "pick":
            return self.execute_pick( target )
        if command in ("place", "pour"):
            return self.execute_pick_and_place( command )
        if command in ("place_shelf"):
            return self.execute_place_shelve(command, target)
        if command in ("give"):
            return self.execute_give()
        if command in ("grab"):
            return self.execute_grab()
        if command in ("observe"):
            return self.execute_observe()
        if command in ("move_arm"):
            return self.go_to_joint_position( target )
        return -1

    def execute_pick(self, target: str) -> int:
        """Method to execute the pick action"""
        rospy.loginfo(f"[INFO] Picking {target}")
        if not self.FAKE_TASKS:
            self.manipulation_client.send_goal(
                        manipulationPickAndPlaceGoal(object_name = target),
                )
            self.manipulation_client.wait_for_result()
            result = self.manipulation_client.get_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]

    def execute_pick_and_place(self, target) -> int:
        """Method to call the pick and place action server"""

        def manipulation_goal_feedback(feedback_msg):
            pass
        
        
        rospy.loginfo(f"[INFO] Manipulation Target: {target}")
        
        if not self.FAKE_TASKS:
            self.manipulation_client.send_goal(
                        manipulationPickAndPlaceGoal(object_name = target),
                        feedback_cb=manipulation_goal_feedback,
                )

            self.manipulation_client.wait_for_result()
            result = self.manipulation_client.get_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        
    def execute_place_shelve(self, target: str, command : str) -> int:

        def manipulation_goal_feedback(feedback_msg):
            pass

        rospy.loginfo(f"[INFO] Manipulation Target: {target}")

        
        min_height, max_height = command.split(" ")
        min_height, max_height = float(min_height), float(max_height)

        if not self.FAKE_TASKS:
            self.manipulation_client.send_goal(
                        manipulationPickAndPlaceGoal(object_name = target, plane_min_height = min_height, plane_max_height = max_height),
                        feedback_cb=manipulation_goal_feedback,
                )

            self.manipulation_client.wait_for_result()
            result = self.manipulation_client.get_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.result else TasksManipulation.STATE["EXECUTION_ERROR"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def execute_give(self) -> int:
        rospy.loginfo(f"[INFO] Giving")
        if not self.FAKE_TASKS:
            # execute give
            rospy.loginfo("[INFO] Giving...")
            self.go_to_joint_position("RECEIVE_JOINT_POSITION")
            rospy.sleep(1)
            rospy.loginfo("[INFO] Opening gripper")
            self.gripper_service(True)
            rospy.sleep(3)
            rospy.loginfo("[INFO] Closing gripper")
            self.gripper_service(False)
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def execute_grab(self) -> int:
        rospy.loginfo(f"[INFO] Grabbing")
        if not self.FAKE_TASKS:
            # execute grab
            rospy.loginfo("[INFO] Moving to grab position")
            self.go_to_joint_position("RECEIVE_JOINT_POSITION")
            rospy.sleep(1)
            rospy.loginfo("[INFO] Opening gripper")
            self.gripper_service(True)
            rospy.sleep(3)
            rospy.loginfo("[INFO] Closing gripper")
            self.gripper_service(False)
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def execute_observe(self) -> int:
        rospy.loginfo(f"[INFO] Observing")
        if not self.FAKE_TASKS:
            # execute observe
            rospy.loginfo("[INFO] Moving to observe position")
            self.move_arm_client.send_goal(
                MoveJointGoal(predefined_position="observe"),
            )
            self.move_arm_client.wait_for_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def move_arm_joints(self, target_x: int, target_y: int, position: str = "") -> int:
        if not self.FAKE_TASKS:
            """Method to move the arm joints"""
            if position != "":
                self.move_arm_client.send_goal(
                    MoveJointGoal(predefined_position = position)
                )
            else:
                self.move_arm_client.send_goal(
                    MoveJointGoal(target_delta_x = target_x, target_delta_y = target_y)
                )
            self.move_arm_client.wait_for_result()
            result = self.move_arm_client.get_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if result.success else TasksManipulation.STATE["EXECUTION_ERROR"]
        return TasksManipulation.STATE["EXECUTION_SUCCESS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        if not self.FAKE_TASKS:
            self.manipulation_client.cancel_all_goals()
            rospy.loginfo("Command canceled Manipulation")
        else:
            rospy.loginfo("Command canceled Manipulation")
            
    def go_to_joint_position(self, position: str) -> int:
        """Method to move the arm to a predefined joint position"""
        if not self.FAKE_TASKS:
            self.move_arm_client.send_goal(
                MoveJointGoal(predefined_position = position, speed = 0.2)
            )
            self.move_arm_client.wait_for_result()
            # joints_target = MANIPULATION_POSITIONS[position]
            # self.moveARM(joints_target, 0.2)
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]

    def moveARM(self, joints, speed, enable_octomap = True):
        if enable_octomap:
            rospy.loginfo("[WARNING] MOVING ARM WITH OCTOMAP DISABLED")
            self.toggle_octomap(False)
        ARM_JOINTS = rospy.get_param("ARM_JOINTS", ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"])
        joint_state = JointState()
        joint_state.name = ARM_JOINTS
        joint_state.position = joints
        # set speed
        self.arm_group.set_max_velocity_scaling_factor(speed)
        # set RRTConnect and timeout
        self.arm_group.set_planner_id("RRTConnect")
        self.arm_group.set_planning_time(20)
        # planning attempts
        self.arm_group.set_num_planning_attempts(10)
        self.arm_group.go(joint_state, wait=True)
        self.arm_group.stop()
        if enable_octomap:
            self.toggle_octomap(True)
    
    def open_gripper(self) -> int:
        """Method to open the gripper"""
        if not self.FAKE_TASKS:
            self.gripper_service("open")
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def close_gripper(self) -> int:
        """Method to close the gripper"""
        if not self.FAKE_TASKS:
            self.gripper_service("close")
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        
    def move_xyz(self, x = 0, y = 0, z = 0, move_x = False, move_y = False, move_z = False) -> int:
        """Method to move the arm to a predefined joint position"""
        if not self.FAKE_TASKS:
            move_pose_ = moveXYZ()
            move_pose_.x = x
            move_pose_.y = y
            move_pose_.z = z
            move_pose_.move_x = move_x
            move_pose_.move_y = move_y
            move_pose_.move_z = move_z

            resp = self.move_pose_client(move_pose_)
            return TasksManipulation.STATE["EXECUTION_SUCCESS"] if resp.success else TasksManipulation.STATE["EXECUTION_ERROR"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
    
    def moveNAV(self) -> int:
        """Method to move the arm to a predefined joint position"""
        if not self.FAKE_TASKS:
            joints_target = MANIPULATION_POSITIONS["NAV_JOINT_POSITION"]
            self.move_arm_client.send_goal(
                MoveJointGoal(joints_target = joints_target)
            )
            self.move_arm_client.wait_for_result()
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]
        else:
            return TasksManipulation.STATE["EXECUTION_SUCCESS"]


if __name__ == "__main__":
    try:
        TasksManipulation()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
