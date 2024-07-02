#!/usr/bin/env python3

"""
This script manages the implementation of each Nav tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_geometry_msgs import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult
from frida_navigation_interfaces.srv import CreateGoal, CreateGoalRequest, CreateGoalResponse
from frida_navigation_interfaces.msg import moveActionAction, moveActionGoal, moveActionResult, moveActionFeedback
from std_srvs.srv import SetBool
import math

import tf2_ros
import tf.transformations as transformations

NAV_SERVER = "/navServer"
MOVE_BASE_SERVER = "/move_base"
LOCATION_TOPIC = "/robot_pose"
APPROACH_SERVER = "/moveServer"

class TasksNav:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["go", "follow", "stop", "approach", "remember"]

    def __init__(self, fake = False) -> None:
        self.FAKE_TASKS = fake
        if not self.FAKE_TASKS:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            #self.nav_client = actionlib.SimpleActionClient(NAV_SERVER, navServAction)
            self.move_base_client = actionlib.SimpleActionClient(MOVE_BASE_SERVER, MoveBaseAction)
            self.approach_client = actionlib.SimpleActionClient(APPROACH_SERVER, moveActionAction)
            # self.map_pose_transformer = rospy.ServiceProxy("/create_goal", CreateGoal)
            self.follow_person_toggle = rospy.ServiceProxy("/change_follow_person_state", SetBool)
            self.test_pose_pub = rospy.Publisher("/nav_test_pose_task_manager", PoseStamped, queue_size=1)
            
            rospy.loginfo("[INFO] Waiting for nav server")
            #if not self.nav_client.wait_for_server(timeout=rospy.Duration(5.0)):
             #   rospy.logerr("Nav server not initialized")
            rospy.loginfo("[INFO] Waiting for move base server")
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("[INFO] Move Base server not initialized")
            # rospy.loginfo("[INFO] Waiting for map pose transformer")
            # if not self.map_pose_transformer.wait_for_service(timeout=rospy.Duration(5.0)):
            #     rospy.logerr("[INFO] Map pose transformer not initialized")
                # rospy.loginfo("[INFO] Waiting for approach person service")
                # if not self.approach_client.wait_for_server(timeout=rospy.Duration(5.0)):
                #     rospy.logerr("[INFO] Approach person server not initialized")
        else:
            rospy.loginfo("[INFO] Fake Nav Task Manager initialized")
        
        rospy.loginfo("[SUCCESS] Nav Task Manager initialized")
        self.past_location = None

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] Nav Command")

        if command == "go":
            return self.go_place(target)
        if command == "remember":
            return self.store_current_location()
        if command == "follow":
            return self.follow_person()
        if command == "stop_follow":
            return self.stop_follow_person()
        if command == "approach":
            return self.approach_pose(target)

        return TasksNav.STATE["EXECUTION_ERROR"]

    def go_place(self, target: str) -> int:
        """Action to move the robot to a specific location"""
        if not self.FAKE_TASKS:
            if target == "past location":
                if self.past_location is None:
                    rospy.logerr("No past location stored")
                    return TasksNav.STATE["EXECUTION_ERROR"]
                
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.past_location

                self.move_base_client.send_goal(goal)
                self.move_base_client.wait_for_result()
                rospy.loginfo("[SUCCESS] Arrived at past location")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            elif target == "back location":
                rospy.loginfo("[INFO] Going back to back location")
                approach_goal = moveActionGoal()
                approach_goal.goal_type = moveActionGoal.BACKWARD
                self.approach_client.send_goal(approach_goal)
                self.approach_client.wait_for_result()
                #self.go_place("past location")
                rospy.loginfo("[SUCCESS] Arrived at back location")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
                
            # Move to room location
            try:
                goal = navServGoal()
                goal.target_location = target

                self.nav_client.send_goal(goal)
                self.nav_client.wait_for_result()
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("Location not found")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[INFO] Going to past location" if target == "past location" else f"Going to {target}")
            rospy.loginfo("[SUCCESS] Arrived at past location" if target == "past location" else f"Arrived at {target}")
            return TasksNav.STATE["EXECUTION_SUCCESS"]

    def go_pose(self, target: PoseStamped) -> int:
        """Action to move the robot to a specific location"""
        if not self.FAKE_TASKS:
            goal = MoveBaseGoal()
            if goal.target_pose.header.frame_id != "map":
                # Transform pose to the map frame
                rospy.loginfo("[INFO] Transforming pose")
                
                pass
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = target.pose

            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        else:
            rospy.loginfo("[INFO] Going to pose")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
    
    def approach_pose(self, target: PointStamped) -> int:
        """Action to approach a specific location"""
        if not self.FAKE_TASKS:
            # transform to base_footprint
            transformed_point = self.tf_buffer.transform(target, "base_footprint")
            # calculate angle to face the target
            angle = math.atan2(transformed_point.point.y, transformed_point.point.x)
            transformed_target = PoseStamped()
            transformed_target.header.frame_id = "base_footprint"
            transformed_target.pose.position.x = 0
            transformed_target.pose.position.y = 0
            transformed_target.pose.position.z = 0
            quaternion = transformations.quaternion_from_euler(0, 0, angle*0.8)
            transformed_target.pose.orientation.x = quaternion[0]
            transformed_target.pose.orientation.y = quaternion[1]
            transformed_target.pose.orientation.z = quaternion[2]
            transformed_target.pose.orientation.w = quaternion[3]
            map_target = self.tf_buffer.transform(transformed_target, "map")
            move_goal = MoveBaseGoal()
            move_goal.target_pose = map_target
            self.test_pose_pub.publish(map_target)
            self.move_base_client.send_goal(move_goal)
            self.move_base_client.wait_for_result()
            
            rospy.loginfo("[INFO] Approaching pose")
            
            approach_goal = moveActionGoal()
            approach_goal.goal_type = moveActionGoal.FORWARD
            self.approach_client.send_goal(approach_goal)
            self.approach_client.wait_for_result(rospy.Duration.from_sec(20.0))
            
            rospy.loginfo("[SUCCESS] Arrived at pose")
            
            return TasksNav.STATE["EXECUTION_SUCCESS"]
        else:
            rospy.loginfo("[INFO] Approaching pose")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
    
    def store_current_location(self) -> int:
        """Method to retrieve the current location of the robot"""
        if not self.FAKE_TASKS:
            try:
                self.past_location = rospy.wait_for_message(LOCATION_TOPIC, Pose, timeout=3.0)
                rospy.loginfo("[SUCCESS] Current location stored")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("[SUCCESS] Unable to store current location")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[SUCCESS] Current location stored")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
    
    def follow_person(self) -> int:
        """Method to follow a person"""
        if not self.FAKE_TASKS:
            try:
                self.follow_person_toggle(True)
                rospy.loginfo("[SUCCESS] Starting Following person")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("[SUCCESS] Unable to follow person")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[SUCCESS] Starting Following person")
            return TasksNav.STATE["EXECUTION_SUCCESS"]
    
    def stop_follow_person(self) -> int:
        """Method to stop following a person"""
        if not self.FAKE_TASKS:
            try:
                self.follow_person_toggle(False)
                rospy.loginfo("[SUCCESS] Stopping Following person")
                return TasksNav.STATE["EXECUTION_SUCCESS"]
            except rospy.ROSException:
                rospy.logerr("[SUCCESS] Unable to stop following person")
                return TasksNav.STATE["EXECUTION_ERROR"]
        else:
            rospy.loginfo("[SUCCESS] Stopping Following person")
            return TasksNav.STATE["EXECUTION_SUCCESS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        if not self.FAKE_TASKS:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("[INFO] Command canceled Nav")
    
    def cancel_goals(self) -> None:
        """Method to cancel all goals"""
        if not self.FAKE_TASKS:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("[INFO] Goals canceled Nav")

if __name__ == "__main__":
    try:
        TasksNav()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'[ERROR] Error: {e}')
