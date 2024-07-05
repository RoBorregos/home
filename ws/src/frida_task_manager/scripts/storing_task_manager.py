#!/usr/bin/env python3

"""
Task manager for the Storing task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import copy

### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult

### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = True
FAKE_MANIPULATION = True
FAKE_HRI = True
FAKE_VISION = True



AREAS = ["nav", "manipulation", "hri", "vision"]
VISION_AVAILABLE_MODES = ["get_shelves", "moondream", "robust"]

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    
    TASK_STATES = {
        "PRE_TABLE_POSITION": 0,
        "APPROACH_TABLE": 1,
        "PICK_OBJECT": 2,
        "PRE_SHELVE_POSITION": 4,
        "ANALYZE_SHELVE": 5,
        "APPROACH_SHELVE": 6,
        "GET_SHELVE_CATEGORIES": 7,
        "PLACE": 8,
        "SHUTDOWN": 9
    }
    
    STATE_ENUM = {
        "IDLE": 0,
        "RECEIVE_COMMANDS": 1,
        "EXECUTING_COMMANDS": 2,
        "STOPPING": 3,
        "ERROR": 4,
        "SHUTDOWN": 5
    }

    COMMANDS_CATEGORY = {
        "nav" : ["go", "follow", "stop", "approach", "remember", "go_pose", "stop_follow"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour", "observe"],
        "hri" : ["ask", "interact", "feedback", "analyze_objects"],
        "vision" : ["find", "identify", "count", "get_bag", "get_shelves"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)
        
        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation(fake=FAKE_MANIPULATION)
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
            self.vision_mode = "robust"
            if self.vision_mode not in VISION_AVAILABLE_MODES:
                rospy.logerr("[ERROR] Invalid vision mode")
                return
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")   
        
        self.current_state = TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]
        self.current_command = None
        self.perceived_information = ""

        self.run()

    def execute_command(self, command: Command) -> int:
        """Method for executing a single command inside its area submodule"""

        rospy.loginfo(f"[INFO] Executing command: {command.action} -> {command.complement}")

        task_result = 0
        for area in AREAS:
            if command.action in TaskManagerServer.COMMANDS_CATEGORY[area] and AREA_ENABLED[area]:
                task_result = self.subtask_manager[area].execute_command(
                    command.action, command.complement, self.perceived_information
                )

        if task_result == -1 or task_result == 0:
            rospy.logerr("[ERROR] Error in task execution")
            return TaskManagerServer.STATE_ENUM["ERROR"]

        self.perceived_information += f"{command.action} {command.complement} {task_result} "
        return TaskManagerServer.STATE_ENUM["EXECUTING_COMMANDS"]

    def cancel_command(self) -> None:
        """Method to cancel the current command"""
        for area in AREAS:
            if self.current_command in TaskManagerServer.COMMANDS_CATEGORY[area]:
                self.subtask_manager[area].cancel_command()

    def run(self) -> None:
        """Main loop for the task manager"""
        self.shelf_list = None
        self.shelf_category = {}
        self.shelf_heights = [0.0, 0.6, 0.95, 1.30]
        self.picked_object = None

        def lower_bound(arr, x):
            left = 0
            right = len(arr)-1
            
            while left <= right:
                mid = left + (right - left)//2
                if arr[mid] == x or mid == len(arr) - 1 or arr[mid] < x < arr[mid+1]:
                    return mid
                
                if arr[mid] < x:
                    left = mid + 1
                else:
                    right = mid - 1
                
            return left


        while not rospy.is_shutdown():
            if self.current_state == TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]:
                result = self.execute_command(Command(action="go", complement="kitchen pre_table"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["APPROACH_TABLE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["APPROACH_TABLE"]:
                result = self.execute_command(Command(action="approach", complement="kitchen table"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["PICK_OBJECT"]
            elif self.current_state == TaskManagerServer.TASK_STATES["PICK_OBJECT"]:
                # TODO: Save the picked object label. Get in the results? or do it without using the execute_command method 
                rospy.loginfo("[INFO] Picking object")
                self.picked_object = self.subtask_manager["vision"].get_object()
                if self.picked_object == self.subtask_manager["vision"].no_objects_str:
                    rospy.logerr("[ERROR] No object found")
                    self.current_state = TaskManagerServer.TASK_STATES["SHUTDOWN"]
                    continue

                result = self.execute_command(Command(action="pick", complement=self.picked_object))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["PRE_SHELVE_POSITION"]
            elif self.current_state == TaskManagerServer.TASK_STATES["PRE_SHELVE_POSITION"]:
                result = self.execute_command(Command(action="go", complement="kitchen pre_shelve"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]:
                if (self.shelf_list is not None or self.vision_mode == "robust"):
                    self.current_state = TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]
                    continue

                rospy.loginfo("[INFO] Analyzing shelves")
                
                if self.vision_mode == "get_shelves":
                    self.shelf_list = self.subtask_manager["vision"].get_shelves()
                elif self.vision_mode == "moondream":
                    self.shelf_list = self.subtask_manager["vision"].get_shelves_moondream()
                else:
                    self.shelf_list = []
                    for height in self.shelf_heights:
                        result = self.subtask_manager["manipulation"].move_arm_joints(0, height)
                        if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                            rospy.logerr("[ERROR] Error in task execution")
                            return
                        labels = self.subtask_manager["vision"].get_objects()

                        if labels is not None:
                            self.shelf_list += labels
                            
                        if FAKE_VISION:
                            break

                if len(self.shelf_list) == 0:
                    rospy.logerr("[ERROR] No shelves found")
                    break

                rospy.loginfo(f"[INFO] Shelves: {self.shelf_list}")
                self.current_state = TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]:
                result = self.execute_command(Command(action="approach", complement="kitchen shelve"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["GET_SHELVE_CATEGORIES"] if self.vision_mode != "robust" else TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]
                self.vision_mode = ""
            elif self.current_state == TaskManagerServer.TASK_STATES["GET_SHELVE_CATEGORIES"]:
                rospy.loginfo("[INFO] Getting shelve categories")
                if self.shelf_category == {}:
                    for i in range(len(self.shelf_list)):
                        category = self.subtask_manager["hri"].get_items_category(self.shelf_list[i]["objects"])
                        index = lower_bound(self.shelf_heights, self.shelf_list[i]["height"])
                        if index == 0:
                            rospy.logerr(f"[ERROR] Invalid shelve height for {self.shelf_list[i]["objects"]}")
                            continue
                        self.shelf_category[category] = self.shelf_heights[index]
                    rospy.loginfo(f"[INFO] Categories: {self.shelf_category}")
                self.picked_object_category = self.subtask_manager["hri"].get_object_category([self.picked_object])
                self.current_state = TaskManagerServer.TASK_STATES["PLACE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["PLACE"]:
                rospy.loginfo("[INFO] Placing object in the shelve")
                if self.picked_object_category in self.shelf_category:
                    rospy.loginfo(f"[INFO] Placing object in the {self.picked_object_category} shelve")
                    self.subtask_manager["manipulation"].move_arm_joints(0, self.shelf_category[self.picked_object_category])
                    result = self.execute_command(Command(action="place"))
                    if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                        rospy.logerr("[ERROR] Error in task execution")
                        break
                else:
                    heights = copy.deepcopy(self.shelf_heights)
                    for key in self.shelf_category:
                        heights.remove(self.shelf_category[key])

                    if len(heights) > 0 and heights[0] == 0.0:
                        heights.pop(0)
                    
                    if len(heights) > 0:
                        rospy.loginfo(f"[INFO] Placing object in empty shelve at {heights[0]} heights")
                        result = self.subtask_manager["manipulation"].move_arm_joints(0, heights[0])
                        if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                            rospy.logerr("[ERROR] Error in task execution")
                            break
                    else:
                        rospy.logerr("[ERROR] No available space in the shelve")
                        break
                self.current_state = TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]    
            if self.current_state == TaskManagerServer.TASK_STATES["SHUTDOWN"]:
                rospy.loginfo("[INFO] Returning to initial position...")
                self.subtask_manager["hri"].speak("I have finished my tasks, I'm going to rest now")
                break
            self._rate.sleep()
        rospy.loginfo("[SUCCESS] Task Finished")

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
