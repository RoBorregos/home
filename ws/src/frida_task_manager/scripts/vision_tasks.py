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
from frida_manipulation_interfaces.msg import objectDetectionArray, objectDetection
from frida_vision_interfaces.srv import Pointing
#from frida_vision_interfaces.srv import ShelfDetection

import math

DIRECTION_BAG_SERVER = "/get_bag_direction"
POINTING_BAG_SERVER = "/detectPointingObject"

POINTING_ACTIVE = False
CARRY = True


DETECTION_TRIES = 3

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
            if POINTING_ACTIVE:
                self.bag_client = actionlib.SimpleActionClient(POINTING_BAG_SERVER, DetectPointingObjectAction)
                if not self.bag_client.wait_for_server(timeout=rospy.Duration(5.0)):
                    rospy.logerr("Bag server not initialized")
            elif CARRY:
                self.bag_direction_client = rospy.ServiceProxy('/person_pointing', Pointing)
                if not self.bag_direction_client.wait_for_service(timeout=rospy.Duration(5.0)):
                    rospy.logerr("Bag direction service not initialized")
            # else:
                # self.shelf_client = rospy.ServiceProxy('/shelf_detector', ShelfDetection)
                # if not self.shelf_client.wait_for_service(timeout=rospy.Duration(5.0)):
                #     rospy.logerr("Shelf detection service not initialized")
            
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
        if command == "get_bag_direction":
            self.get_bag_direction()

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
            if POINTING_ACTIVE:
                # Dummy values
                result = DetectPointingObjectResult(result = True, label=1, labelText="bag", point3D=PoseStamped(header="base_link", pose=Pose(position=(10, 20, 30), orientation=(70, 60, 50, 1))))

                if result.result:
                    rospy.loginfo(f"[SUCCESS] Result: {result}")
                    rospy.loginfo(f"[SUCCESS] Pose Result: {result.point3D}")
                    self.bag_information["id"] = result.label
                    self.bag_information["name"] = result.labelText
                    self.bag_information["PoseStamped"] = result.point3D
                    return TasksVision.STATE["EXECUTION_SUCCESS"]
            else:
                result = self.bag_direction_client()
                if result.data != 0:
                    directions_dict = {
                        0: "failed",
                        1: "left",
                        2: "right",
                    }
                    self.bag_information["id"] = result.data
                    self.bag_information["name"] = directions_dict[result.data]

        return TasksVision.STATE["EXECUTION_ERROR"]
    
    def get_bag_direction(self) -> str:
        """Method to get the bag direction"""
        rospy.loginfo("[INFO] Getting the bag direction")
        if not self.FAKE_TASKS:
            result = self.bag_direction_client(True, 5)
            directions_dict = {
                0: "failed",
                1: "left",
                2: "right",
            }
            self.bag_information["id"] = result.result
            self.bag_information["name"] = directions_dict[result.result]
            return self.bag_information["name"]
        else:
            # Dummy values
            return "right"
        return TasksVision.STATE["EXECUTION_ERROR"]
    
    def get_bag_information(self) -> dict:
        """Method to get the bag information"""
        return self.bag_information["id"], self.bag_information["name"], self.bag_information["PoseStamped"]
    
    def get_object(self) -> str:
        """Method to get the object"""
        if self.FAKE_TASKS:
            return "sample_object"
        for i in range(DETECTION_TRIES):
            rospy.loginfo("[INFO] Getting Detections")
            try: 
                detections = rospy.wait_for_message("/detections", objectDetectionArray, timeout=10.0)
                closest_distance = 10000
                closest_index = 0
                for i, detection in enumerate(detections.detections):
                    # return closest object
                    #point3d is pointstamped
                    point3D = detection.point3D.point
                    distance = math.sqrt(point3D.x**2 + point3D.y**2 + point3D.z**2)
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_index = i
                return detections.detections[closest_index].labelText
                    
            except rospy.exceptions.ROSException:
                print("No objects detected")
        
    def get_shelves(self) -> list:
        """Method to get the shelves"""
        # returns list with [shelve["shelve_number"], shelve["category"], shelve["height"]]
        if self.FAKE_TASKS:
            newShelve = {}
            newShelve["shelve_number"] = 1
            newShelve["category"] = "sample_category1"
            newShelve["height"] = 0.30
            newShelve2 = {}
            newShelve2["shelve_number"] = 2
            newShelve2["category"] = "sample_category2"
            newShelve2["height"] = 0.40
            return [newShelve, newShelve2]
        else:
            shelf_result = self.shelf_client(True)
            shelf = shelf_result.shelf
            shelf_list = []
            for level in shelf.levels:
                newShelve = {}
                # newShelve["shelve_number"] = level.label
                newShelve["objects"] = level.objects
                newShelve["height"] = level.height
                shelf_list.append(newShelve)
            return shelf_list
        return []
        



if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
