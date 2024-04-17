#!/usr/bin/env python3

"""
This script manages the implementation of each Vision tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from frida_vision_interfaces.msg import DetectPointingObjectAction, DetectPointingObjectGoal, DetectPointingObjectResult, DetectPointingObjectFeedback

POINTING_BAG_SERVER = "/detectPointingObject"

class TasksVision:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["get_bag"]

    def __init__(self, fake = False) -> None:
        
        self.FAKE_TASKS = fake
        # Context information
        self.bag_information = {
            "id": 0,
            "name": "",
            "PoseStamped": PoseStamped()
        }
        
        if not self.FAKE_TASKS:
            rospy.loginfo("[INFO] Waiting for bag server")
            self.bag_client = actionlib.SimpleActionClient(POINTING_BAG_SERVER, DetectPointingObjectAction)
            if not self.bag_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("Bag server not initialized")
        else:
            rospy.loginfo("Fake Vision Task Manager initialized")
        
        rospy.loginfo("[SUCCESS] Vision Task Manager initialized")

    def execute_command(self, command: str, target: str, info: str) -> int:
        """Method to execute each command"""
        rospy.loginfo("[INFO] Nav Command")

        if command == "get_bag":
            self.get_bag()
            # set dummy values
            # self.bag_information["id"] = 1
            # self.bag_information["name"] = "bag"
            # # set dummy pose with dummy values
            # self.bag_information["PoseStamped"] = PoseStamped(header="zed2_camera_link", 
            #                                                   pose=Pose(position=Point(10,20,30),
            #                                                             orientation=Quaternion(0,0,0,1))
            # )

        return TasksVision.STATE["EXECUTION_ERROR"]

    def get_bag(self) -> int:
        """Method to get the bag"""
        rospy.loginfo("[INFO] Getting the bag")

        if not self.FAKE_TASKS:
            goal = DetectPointingObjectGoal(waiting_time=5)
            self.bag_client.send_goal(goal)
            self.bag_client.wait_for_result()

            result = self.bag_client.get_result()
        else:
            # Dummy values
            result = DetectPointingObjectResult(result = True, label=1, labelText="bag", point3D=PoseStamped(header="base_link", pose=Pose(position=(10, 20, 30), orientation=(70, 60, 50, 1))))

        if result.result:
            rospy.loginfo(f"[SUCCESS] Result: {result}")
            rospy.loginfo(f"[SUCCESS] Pose Result: {result.point3D}")
            self.bag_information["id"] = result.label
            self.bag_information["name"] = result.labelText
            self.bag_information["PoseStamped"] = result.point3D
            return TasksVision.STATE["EXECUTION_SUCCESS"]

        return TasksVision.STATE["EXECUTION_ERROR"]
    
    def get_bag_information(self) -> dict:
        """Method to get the bag information"""
        return self.bag_information["id"], self.bag_information["name"], self.bag_information["PoseStamped"]

if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
