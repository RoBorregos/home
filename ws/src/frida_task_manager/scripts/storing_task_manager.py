#!/usr/bin/env python3

"""
Task manager for the Storing task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib
import copy
import math
### ROS messages
from std_msgs.msg import String, Bool
from frida_hri_interfaces.msg import Command, CommandList
from frida_hri_interfaces.msg import ConversateAction, ConversateFeedback, ConversateGoal, ConversateResult
from frida_navigation_interfaces.msg import navServAction, navServFeedback, navServGoal, navServResult


### Python submodules
from hri_tasks import TasksHRI
from manipulation_tasks import TasksManipulation
from nav_tasks import TasksNav
from vision_tasks import TasksVision

NAV_ENABLED = True
MANIPULATION_ENABLED = True
CONVERSATION_ENABLED = True
VISION_ENABLED = True

FAKE_NAV = False
FAKE_MANIPULATION = False
FAKE_HRI = False
FAKE_VISION = False

SHELF_SIZE = 0.35
TARGET_SHELVE_APPROACH = 0.4

# For Storing Groceries
# Constants
ARM_MAX_HEIGHT = 1.3
ARM_MIN_HEIGHT = 1.0
ARM_MAX_ANGLE = 45
ARM_MIN_ANGLE = -45
DISTANCE_TO_SHELF = TARGET_SHELVE_APPROACH

AREAS = ["nav", "manipulation", "hri", "vision"]
VISION_AVAILABLE_MODES = ["fast_execution", "robust"]

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
        "DEPROACH_TABLE": 3,
        "PRE_SHELVE_POSITION": 4,
        "ANALYZE_SHELVE": 5,
        "APPROACH_SHELVE": 6,
        "GET_SHELVE_CATEGORIES": 7,
        "PLACE": 8,
        "DEPROACH_SHELVE": 11,
        "SHUTDOWN": 9,
        "DEPROACH": 10
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
        "nav" : ["go", "follow", "stop", "approach", "remember", "go_pose", "stop_follow", "deproach"],
        "manipulation" : ["pick", "place", "grasp", "give", "open", "close", "pour", "observe", "place_shelf", "move_arm"],
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
        
        # self.current_state = TaskManagerServer.TASK_STATES["PRE_TABLE_POSITION"]
        self.current_state = TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]
        self.current_command = None
        self.perceived_information = ""

        self.run()

    def get_height_angle_for_shelve(self, target_shelf_height):
        def find_angle_from_height(height):
            return (height - ARM_MIN_HEIGHT) / (ARM_MAX_HEIGHT - ARM_MIN_HEIGHT) * (ARM_MAX_ANGLE - ARM_MIN_ANGLE) + ARM_MIN_ANGLE
        h_step = 0.01
        curr_h = ARM_MIN_HEIGHT
        result_height = None
        result_angle = None
        lowest_diff = 1000
        while curr_h <= ARM_MAX_HEIGHT:
            curr_a = find_angle_from_height(curr_h)
            m = math.tan(math.radians(curr_a))
            b = curr_h
            x = DISTANCE_TO_SHELF
            y_target = target_shelf_height
            y = m*x + b
            diff = abs(y - y_target)
            if diff < lowest_diff:
                lowest_diff = diff
                result_height = curr_h
                result_angle = curr_a
            else:
                break
            curr_h += h_step
        if lowest_diff > 0.1:
            return None, None
        return result_height, result_angle

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

        self.picked_object = "milk"

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
                result = self.execute_command(Command(action="move_arm", complement="NAV_JOINT_POSITION"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
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
                rospy.sleep(2)
                self.picked_object = self.subtask_manager["vision"].get_object()
                if self.picked_object == self.subtask_manager["vision"].no_objects_str:
                    rospy.logerr("[ERROR] No object found")
                    self.current_state = TaskManagerServer.TASK_STATES["SHUTDOWN"]
                    continue

                result = self.execute_command(Command(action="pick", complement=self.picked_object))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
                    break
                self.current_state = TaskManagerServer.TASK_STATES["DEPROACH_TABLE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["DEPROACH_TABLE"]:
                rospy.loginfo("[INFO] Deproaching...")
                result = self.execute_command(Command(action="deproach"))
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
                if ((self.shelf_list is not None and self.shelf_category == {}) or self.vision_mode == "robust"):
                    self.current_state = TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]
                    continue

                rospy.loginfo("[INFO] Analyzing shelves")
                
                if self.vision_mode == "fast_execution":
                    self.shelf_list = self.subtask_manager["vision"].get_shelves()
                else:
                    result = self.subtask_manager["manipulation"].go_to_joint_position("shelves")
                    if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                        rospy.logerr("[ERROR] Error in task execution")
                        return
                    rospy.loginfo("[INFO] In shelves position")
                    self.shelf_list = []
                    for idx, height in enumerate(self.shelf_heights):
                        rospy.loginfo("[INFO] Analyzing height {} index: {}...".format(height, idx))
                        if height == 0.0:
                            continue
                        # rospy.loginfo("[INFO] Analyzing height {}...".format(height))

                        # Get angel vision from the height and the target_approach distance
                        target_height, target_angle = self.get_height_angle_for_shelve(height + SHELF_SIZE/2)
                        rospy.loginfo(f"[INFO] Target height: {target_height}, Target angle: {target_angle}")

                        if target_height is None or target_angle is None:
                            rospy.logerr("[ERROR] Invalid target height or angle")
                            return
                        
                        result = self.subtask_manager["manipulation"].move_xyz(z = target_height, move_z = True)
                        if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                            rospy.logerr("[ERROR] Error in task execution")
                            return
                        
                        result = self.subtask_manager["manipulation"].move_arm_joints(0, -int(target_angle))
                        rospy.sleep(2)

                        rospy.loginfo("[INFO] Getting category from height {}...".format(height))
                        category = self.subtask_manager["vision"].get_shelve_moondream()
                        if category.lower() != "empty":
                            # self.shelf_list.append({"objects": category, "height": height})
                            self.shelf_category[category] = height
                        rospy.loginfo(f"[INFO] Category: {category}")

                        if FAKE_VISION:
                            break

                        result = self.subtask_manager["manipulation"].move_arm_joints(0, int(target_angle))
                if len(self.shelf_list) == 0 and self.shelf_category == {}:
                    rospy.logerr("[ERROR] No shelves found")
                    break

                rospy.loginfo(f"[INFO] Shelves: {self.shelf_list}")
                self.current_state = TaskManagerServer.TASK_STATES["APPROACH_SHELVE"] if self.vision_mode == "fast_execution" else TaskManagerServer.TASK_STATES["GET_SHELVE_CATEGORIES"]
            elif self.current_state == TaskManagerServer.TASK_STATES["APPROACH_SHELVE"]:
                rospy.loginfo("[INFO] Approaching shelve...")
                goal = navServGoal()
                goal.goal_type = navServGoal.FORWARD
                goal.target_location = "kitchen shelve"
                goal.target_approach = TARGET_SHELVE_APPROACH
                # self.nav_client.send_goal(goal)
                # self.nav_client.wait_for_result()
                # self.subtask_manager["nav"].nav_client.send_goal(goal)
                # self.subtask_manager["nav"].nav_client.wait_for_result()
                self.current_state = TaskManagerServer.TASK_STATES["GET_SHELVE_CATEGORIES"] if self.vision_mode != "robust" else TaskManagerServer.TASK_STATES["ANALYZE_SHELVE"]
                self.vision_mode = "" if self.vision_mode == "robust" else self.vision_mode
            elif self.current_state == TaskManagerServer.TASK_STATES["GET_SHELVE_CATEGORIES"]:
                rospy.loginfo("[INFO] Getting shelve categories")
                if self.shelf_category == {}:
                    for i in range(len(self.shelf_list)):
                        category = self.subtask_manager["hri"].get_items_category(self.shelf_list[i]["objects"])
                        index = lower_bound(self.shelf_heights, self.shelf_list[i]["height"])
                        if index == 0:
                            # rospy.logerr(f"[ERROR] Invalid shelve height for {self.shelf_list[i]["objects"]}")
                            continue
                        self.shelf_category[category] = self.shelf_heights[index]
                rospy.loginfo(f"[INFO] Categories: {self.shelf_category}")
                self.picked_object_category = self.subtask_manager["hri"].get_object_category(self.picked_object)
                self.current_state = TaskManagerServer.TASK_STATES["PLACE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["PLACE"]:
                rospy.loginfo("[INFO] Placing object in the shelve of category {}...".format(self.picked_object_category))
                if self.picked_object_category in self.shelf_category:
                    # Get angel vision from the height and the target_approach distance
                    target_height, target_angle = self.get_height_angle_for_shelve(self.shelf_category[self.picked_object_category] + SHELF_SIZE/2)
                    rospy.loginfo(f"[INFO] Target height: {target_height}, Target angle: {target_angle}")

                    if target_height is None or target_angle is None:
                        rospy.logerr("[ERROR] Invalid target height or angle")
                        return
                    
                    result = self.subtask_manager["manipulation"].move_xyz(z = target_height, move_z = True)
                    if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                        rospy.logerr("[ERROR] Error in task execution")
                        return
                    
                    result = self.subtask_manager["manipulation"].move_arm_joints(0, -int(target_angle))
                    rospy.sleep(1)
                    heights = "{} {}".format(self.shelf_category[self.picked_object_category] - 0.05, self.shelf_category[self.picked_object_category] + 0.05)
                    result = self.execute_command(Command(action="place_shelf", complement=heights))
                    if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                        rospy.logerr("[ERROR] Error in task execution")
                        break
                else:
                    heights = copy.deepcopy(self.shelf_heights)
                    for key in self.shelf_category:
                        heights.remove(self.shelf_category[key])
                        rospy.loginfo(f"[INFO] Removing height {self.shelf_category[key]}")

                    if len(heights) > 0 and heights[0] == 0.0:
                        heights.pop(0)
                    rospy.loginfo(f"[INFO] Available heights: {heights}")
                    if len(heights) > 0:
                        target_height, target_angle = self.get_height_angle_for_shelve(heights[0] + SHELF_SIZE/2)
                        rospy.loginfo(f"[INFO] Placing object in empty shelve at {heights[0]} heights")
                        result = self.subtask_manager["manipulation"].move_xyz(z = target_height, move_z = True)
                        if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                            rospy.logerr("[ERROR] Error in task execution")
                            break
                        result = self.subtask_manager["manipulation"].move_arm_joints(0, -int(target_angle))
                        rospy.sleep(1)
                        if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                            rospy.logerr("[ERROR] Error in task execution")
                            break
                        heights = "{} {}".format(heights[0] - 0.1, heights[0] + 0.1)
                        result = self.execute_command(Command(action="place_shelf", complement=heights))
                    else:
                        rospy.logerr("[ERROR] No available space in the shelve")
                        break
                self.current_state = TaskManagerServer.TASK_STATES["DEPROACH_SHELVE"]
            elif self.current_state == TaskManagerServer.TASK_STATES["DEPROACH_SHELVE"]:
                rospy.loginfo("[INFO] Deproaching...")
                result = self.execute_command(Command(action="deproach"))
                if result == TaskManagerServer.STATE_ENUM["ERROR"]:
                    rospy.logerr("[ERROR] Error in task execution")
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
