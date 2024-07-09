#!/usr/bin/env python3

"""
This script manages the implementation of each Vision tasks
"""

### Import libraries
import rospy
import actionlib

### ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from frida_vision_interfaces.msg import DetectPointingObjectAction, DetectPointingObjectGoal, DetectPointingObjectResult, DetectPointingObjectFeedback
from frida_vision_interfaces.msg import MoondreamFromCameraAction, MoondreamFromCameraGoal, MoondreamFromCameraResult, MoondreamFromCameraFeedback
from frida_manipulation_interfaces.msg import objectDetection, objectDetectionArray
from frida_vision_interfaces.srv import Pointing, NewHost, NewHostResponse, FindSeat, ShelfDetections
from std_srvs.srv import SetBool
#from frida_vision_interfaces.srv import ShelfDetection
import random
import math

DIRECTION_BAG_SERVER = "/vision/get_bag_direction"
POINTING_BAG_SERVER = "/vision/detectPointingObject"

POINTING_ACTIVE = False
CARRY = False


DETECTION_TRIES = 3
STORE_FACE_SERVICE = "/vision/new_name"
CHECK_PERSON = "/vision/check_person"
FIND_TOPIC = "/vision/find_seat"
POSITION_TOPIC = "/person_pointing"
SHELF_SERVER = "/shelf_detector"
DETECTION_TOPIC = "/detections"
ROOM_DETECTIONS_TOPIC = "/vision/room_detections"
IMAGE_TOPIC = "/zed2/zed_node/rgb/image_rect_color"
MOONDREAM_FROM_CAMERA_AS = "/vision/moondream_from_camera"

################################################################
# MOONDREAM PROMPTS
################################################################

SHELF_PROMPT = "Using only one word, categorize the objects in this shelf"
# Another option with more specific categories
# SHELF_PROMPT = "Using only one word, categorize the objects in this shelf. Answer with 'food', 'tools' or 'toys'"
DRINK_PROMPT = "Answering only 'yes' or 'no', is the person in the image holding a drink?"
PERSON_CHARACTERISTIC_PROMPT = "Answering only 'yes' or 'no', is the person in the image {characteristic}?"
OBJECT_CHARACTERISTIC_PROMPT = "Which is the {characteristic} {object} in the image? Describe it in detail. If there is no object, say: I don't see any object."

class Person:
    xmin = 0
    ymin = 0
    xmax = 0
    ymax = 0
    Point3D = PointStamped()

PERSON_COUNT_START_TOPIC = "/start_counting"
PERSON_COUNT_END_TOPIC = "/end_counting"

class TasksVision:
    """Class to manage the navigation tasks"""
    STATE = {
        "TERMINAL_ERROR": -1,
        "EXECUTION_ERROR": 0,
        "EXECUTION_SUCCESS": 1
    }

    AREA_TASKS = ["get_bag", "wait", "save", "get_shelves", "count"]

    def __init__(self, fake = False) -> None:
        
        self.FAKE_TASKS = fake
        # Context information
        self.bag_information = {
            "id": 0,
            "name": "",
            "PoseStamped": PoseStamped()
        }

        self.no_objects_str = "No objects detected"
        self.n_detections = 5
        self.detected_objects = None
        
        if not self.FAKE_TASKS:
            rospy.loginfo("[INFO] Waiting for bag server")
            if POINTING_ACTIVE:
                self.bag_client = actionlib.SimpleActionClient(POINTING_BAG_SERVER, DetectPointingObjectAction)
                if not self.bag_client.wait_for_server(timeout=rospy.Duration(3.0)):
                    rospy.logerr("Bag server not initialized")
            elif CARRY:
                self.bag_direction_client = rospy.ServiceProxy(POSITION_TOPIC, Pointing)
                if not self.bag_direction_client.wait_for_service(timeout=rospy.Duration(3.0)):
                    rospy.logerr("Bag direction service not initialized")
            else:
                self.shelf_client = rospy.ServiceProxy(SHELF_SERVER, ShelfDetections)
                if not self.shelf_client.wait_for_service(timeout=rospy.Duration(3.0)):
                    rospy.logerr("Shelf detection service not initialized")
            self.cv_bridge = CvBridge()
            self.save_name_call = rospy.ServiceProxy(STORE_FACE_SERVICE, NewHost)
            if not self.save_name_call.wait_for_service(timeout=rospy.Duration(3.0)):
                rospy.logerr("Save name service not initialized")
            self.moondream_from_camera_client = actionlib.SimpleActionClient(MOONDREAM_FROM_CAMERA_AS, MoondreamFromCameraAction)
            if not self.moondream_from_camera_client.wait_for_server(timeout=rospy.Duration(3.0)):
                rospy.logerr("Moondream from camera server not initialized")
            
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
        elif command == "get_bag_direction":
            self.get_bag_direction()
        elif command == "get_object":
            self.get_object()
        elif command == "get_shelves":
            self.get_shelves()
        elif command == "save_face_name":
            self.save_face_name(info)
        elif command == "check_person":
            self.check_person()
        elif command == "find_seat":
            self.find_seat()
        elif command == "identify":
            self.identify(target, info)
        elif command == "cancel":
            self.cancel_command()
        else:
            rospy.logerr("[ERROR] Command not recognized")
            return TasksVision.STATE["TERMINAL_ERROR"]

        return TasksVision.STATE["EXECUTION_ERROR"]

    def get_bag(self) -> int:
        """Method to get the bag"""
        rospy.loginfo("[INFO] Getting the bag")

        if not self.FAKE_TASKS:
            if POINTING_ACTIVE:
                goal = DetectPointingObjectGoal(waiting_time=5)
                self.bag_client.send_goal(goal)
                self.bag_client.wait_for_result()

                result = self.bag_client.get_result()
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
                if result.result:
                    rospy.loginfo(f"[SUCCESS] Result: {result}")
                    rospy.loginfo(f"[SUCCESS] Pose Result: {result.point3D}")
                    self.bag_information["id"] = result.label
                    self.bag_information["name"] = result.labelText
                    self.bag_information["PoseStamped"] = result.point3D
                    return TasksVision.STATE["EXECUTION_SUCCESS"]
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
                detections = rospy.wait_for_message(DETECTION_TOPIC, objectDetectionArray, timeout=10.0)
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
                print(self.no_objects_str)
            
        return self.no_objects_str

    def get_objects(self) -> list:
        """Method to get the labels of the detected objects"""
        obj_labels = []
        for i in range(5):
            self.detected_objects = rospy.wait_for_message(DETECTION_TOPIC, objectDetectionArray)
            for obj in self.detected_objects.objects:
                obj_labels.append(obj.labelText)

        obj_labels = list(set(obj_labels))
            
        return obj_labels
    
    ################################################################
    # STORING GROCERIES
    ################################################################
        
    def get_shelves(self) -> list:
        """Method to get the shelves"""
        # returns list with [shelve["shelve_number"], shelve["category"], shelve["height"]]
        if self.FAKE_TASKS:
            newShelve = {}
            newShelve["shelve_number"] = 1
            newShelve["objects"] = ["sample_object1", "sample_object2", "sample_object3"]
            newShelve["height"] = 0.40
            newShelve2 = {}
            newShelve2["shelve_number"] = 2
            newShelve2["objects"] = ["sample_object1", "sample_object2", "sample_object3"]
            newShelve2["height"] = 0.70
            # newShelve3 = {}
            # newShelve3["shelve_number"] = 3
            # newShelve3["objects"] = ["sample_object1", "sample_object2", "sample_object3"]
            # newShelve3["height"] = 1.0
            
            return [newShelve, newShelve2]
        else:
            shelf_result = self.shelf_client(True)
            shelf = shelf_result.shelf
            shelf_list = []
            
            for level in shelf.levels:
                newShelve = {}
                newShelve["shelve_number"] = level.label
                newShelve["objects"] = level.objects
                newShelve["height"] = level.height
                shelf_list.append(newShelve)
            return shelf_list
        return []
    
    def get_shelve_moondream(self) -> str:
        """Method to get the shelve category using Moondream"""
        if self.FAKE_TASKS:
            return random.choice(["food", "tools", "toys"])
        else:
            goal = MoondreamFromCameraGoal(camera_topic=IMAGE_TOPIC, prompt=SHELF_PROMPT)
            rospy.loginfo("Moondream sent goal for shelf recognition: ", goal)
            self.moondream_from_camera_client.send_goal(goal)
            self.moondream_from_camera_client.wait_for_result()
            result = self.moondream_from_camera_client.get_result()
            rospy.loginfo("Moondream result: ", result)
            return result.response
    
    ################################################################
    # RECEPTIONIST
    ################################################################

    def save_face_name(self, name: str) -> int:
        """Method to save the face name"""
        if self.FAKE_TASKS:
            return TasksVision.STATE["EXECUTION_SUCCESS"]
        
        rospy.loginfo("Save face name")
        try:
            response = self.save_name_call( name )
            if response.success:
                return TasksVision.STATE["EXECUTION_SUCCESS"]
        except rospy.ServiceException:
            rospy.logerr("Service call name failed")

        return TasksVision.STATE["EXECUTION_ERROR"]

    def check_person(self) -> bool:
        """Method to check if a person is detected calling PersonDetection.py"""
        if self.FAKE_TASKS:
            return True
        
        try:
            rospy.wait_for_service(CHECK_PERSON, timeout=5.0)
            check_person = rospy.ServiceProxy(CHECK_PERSON, SetBool)
            response = check_person(True)
            return response.success
        except rospy.ROSException:
            rospy.logerr("Service call check_person failed")
            return False

    def find_seat(self) -> int:
        """Method to find the angle the robot should turn to point the free seat"""
        if self.FAKE_TASKS:
            return 300
        
        try:
            rospy.wait_for_service(FIND_TOPIC, timeout=5.0)
            find_seat = rospy.ServiceProxy(FIND_TOPIC, FindSeat)
            response = find_seat(True)
            if int(response.angle) == -100:
                return 300
            return int(response.angle)
        except rospy.ServiceException:
            rospy.logerr("Service call find_seat failed")
            return 300
    
    def get_people(self, in_room = False) -> list:
        """ Method to get the people detected in the image """
        if self.FAKE_TASKS:
            return [Person(), Person()]
        people = []
        detections = rospy.wait_for_message(ROOM_DETECTIONS_TOPIC if in_room else DETECTION_TOPIC, objectDetectionArray)
        for detection in detections.detections:
            if detection.labelText == "person":
                person = Person()
                person.xmin = detection.xmin
                person.ymin = detection.ymin
                person.xmax = detection.xmax
                person.ymax = detection.ymax
                person.Point3D = detection.point3D
                people.append(person)
        return people
        
    ################################################################
    # GPSR
    ################################################################
    
    def identify(self, target, characteristic) -> bool:
        """ Method to identify a person or object """
        """ This method varies depending on the target and characteristic """
        """ If it is a person, it is necessary to save the person info like point3D to use it if needed to approach them """
        print("Identifying: ", target, characteristic)
        if target == "person":
            self.people_detected = self.get_people()
            if len(self.people_detected) == 0:
                return False
            for person in self.people_detected:
                prompt = PERSON_CHARACTERISTIC_PROMPT.format(characteristic=characteristic)
                print("Checking for person: ", person)
                print(prompt)
                if self.FAKE_TASKS:
                    return True
                moondream_result = self.moondream_from_camera_client.send_goal(MoondreamFromCameraGoal(camera_topic=IMAGE_TOPIC, prompt=prompt))
                if moondream_result.response.lower() == "yes":
                    self.person_info = person
                    return True
            print("No person with the characteristic found")
            return False
        else:
            prompt = OBJECT_CHARACTERISTIC_PROMPT.format(characteristic=characteristic, object=target)
            print("Checking for object: ", target, " with characteristic: ", characteristic)
            print("Prompt: ", prompt)
            if self.FAKE_TASKS:
                return True
            moondream_result = self.moondream_from_camera_client.send_goal(MoondreamFromCameraGoal(camera_topic=IMAGE_TOPIC, prompt=prompt))
            self.identified_object_description = moondream_result.response
            return True
        
            
            
                
            
    def identify_characteristic(self, characteristic) -> str:

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        rospy.loginfo("Command canceled Nav")


if __name__ == "__main__":
    try:
        TasksVision()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
