#!/usr/bin/env python3

"""
Task manager for the Breakfast task of RoboCup @Home 2024
"""

### Import libraries
import rospy
import actionlib

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

AREA_ENABLED = {
    "nav": NAV_ENABLED,
    "manipulation": MANIPULATION_ENABLED,
    "hri": CONVERSATION_ENABLED,
    "vision": VISION_ENABLED
}

class Bag:
    """Class to store the information of the bag"""
    def __init__(self, bag_id: int = 0, name: str = "", PoseStamped = None) -> None:
        self.bag_id = bag_id
        self.name = name
        self.PoseStamped = PoseStamped
    
    def __str__(self) -> str:
        return f"Bag {self.bag_id}: {self.name}, {self.PoseStamped}"
    
    def set_bag_info(self, bag_id: int, name: str, PoseStamped = None) -> None:
        self.bag_id = bag_id
        self.name = name
        self.PoseStamped = PoseStamped
    
    def get_bag_info(self) -> dict:
        return {
            "bag_id": self.bag_id,
            "name": self.name,
            "PoseStamped": self.PoseStamped
        }

class TaskManagerServer:
    """Class to manage different tasks divided by categories"""
    
    TASK_STATES = {
        "GET_BAG": 0,
        "APPROACH_BAG": 1,
        "PICK_BAG": 2,
        "RETURN_TO_OBSERVE": 4,
        "START_FOLLOW_PERSON": 5,
        "FOLLOW_PERSON": 6,
        "GIVE_BAG": 7,
        "RETURN": 8,
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
        "hri" : ["ask", "interact", "feedback"],
        "vision" : ["find", "identify", "count", "get_bag"]
    }

    def __init__(self) -> None:
        self._node = rospy.init_node("task_manager_server")
        self._rate = rospy.Rate(200)
        self.following = False
        rospy.loginfo("STARTED STOP SUB")
        self.stop_following_sub = rospy.Subscriber("/stop_following", Bool, self.stop_following_callback)
        # Creates an empty dictionary to store the subtask manager of each area
        self.subtask_manager = dict.fromkeys(AREAS, None)

        if MANIPULATION_ENABLED:
            self.subtask_manager["manipulation"] = TasksManipulation(fake=FAKE_MANIPULATION)
        if NAV_ENABLED:
            self.subtask_manager["nav"] = TasksNav(fake=FAKE_NAV)
        if VISION_ENABLED:
            self.subtask_manager["vision"] = TasksVision(fake=FAKE_VISION)
        if CONVERSATION_ENABLED:
            self.subtask_manager["hri"] = TasksHRI(fake=FAKE_HRI)
            self.subtask_manager["hri"].speak("Hi, my name is Frida. I'm here to help you with your domestic tasks")

        self.current_state = TaskManagerServer.STATE_ENUM["IDLE"]
        self.current_past_state = None
        self.current_command = None
        self.current_queue = []
        
        # Context information   
        self.perceived_information = ""
        self.bag = Bag()
        
        self.current_state = TaskManagerServer.TASK_STATES["GET_BAG"]
        
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

        if task_result == -1:
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
        while not rospy.is_shutdown():
            if self.current_state == TaskManagerServer.TASK_STATES["GET_BAG"]:
                rospy.loginfo("[INFO] Setting arm to observe")
                self.subtask_manager["manipulation"].go_to_joint_position("OBSERVE_JOINT_POSITION")
                rospy.loginfo("[INFO] Getting bag pointed at...")
                self.subtask_manager["hri"].speak("Please, point at the bag for 5 seconds")
                if self.execute_command(Command(action="get_bag", complement="")) == TaskManagerServer.STATE_ENUM["ERROR"]:
                    continue
                self.subtask_manager["hri"].speak("I got the bag. Thank you!")
                self.bag.set_bag_info(*self.subtask_manager["vision"].get_bag_information())
                # remember location to return
                print("remembering")
                self.execute_command(Command(action="remember", complement=""))
                self.current_state = TaskManagerServer.TASK_STATES["APPROACH_BAG"]
            
            if self.current_state == TaskManagerServer.TASK_STATES["APPROACH_BAG"]:
                rospy.loginfo("[INFO] Approaching bag...")
                self.subtask_manager["hri"].speak(F"I'm going to approach the {self.bag.name} bag")
                
                pose = self.bag.PoseStamped
                rospy.loginfo("[INFO] Setting arm to nav")
                self.subtask_manager["manipulation"].go_to_joint_position("NAV_JOINT_POSITION")
                
                self.subtask_manager["nav"].approach_pose(pose)
                self.current_state = TaskManagerServer.TASK_STATES["PICK_BAG"]
                
            
            if self.current_state == TaskManagerServer.TASK_STATES["PICK_BAG"]:
                rospy.loginfo("[INFO] Picking bag...")
                self.subtask_manager["hri"].speak("I will pick the bag now")
                rospy.loginfo("[INFO] Setting arm to pick")
                self.subtask_manager["manipulation"].go_to_joint_position("PICK_JOINT_POSITION")
                pick_errors = 0
                pick_success = False
                if self.execute_command(Command(action="pick", complement=self.bag.name)) == TaskManagerServer.STATE_ENUM["ERROR"] and pick_errors < 3:
                    pick_errors += 1
                    continue
                else:
                    pick_success = True
                if not pick_success:
                    rospy.logerr("[ERROR] Error picking the bag")
                    self.subtask_manager["hri"].speak("I'm sorry, I couldn't pick the bag, please hand it to my gripper")
                    self.subtask_manager["manipulation"].go_to_joint_position("RECEIVE_JOINT_POSITION")
                    self.subtask_manager["manipulation"].open_gripper()
                    self.subtask_manager["hri"].speak("I'm ready to receive the bag")
                    rospy.sleep(3)
                    self.subtask_manager["manipulation"].close_gripper()
                
                self.subtask_manager["manipulation"].go_to_joint_position("CARRYING_JOINT_POSITION")
                self.current_state = TaskManagerServer.TASK_STATES["RETURN_TO_OBSERVE"]
                
            if self.current_state == TaskManagerServer.TASK_STATES["RETURN_TO_OBSERVE"]:
                rospy.loginfo("[INFO] Returning to observe position...")
                self.subtask_manager["hri"].speak("I'm going back to look for you")
                self.execute_command(Command(action="go", complement="back location"))
                self.current_state = TaskManagerServer.TASK_STATES["START_FOLLOW_PERSON"]
            
            if self.current_state == TaskManagerServer.TASK_STATES["START_FOLLOW_PERSON"]:
                rospy.loginfo("[INFO] Following person...")
                self.subtask_manager["hri"].speak("I'll follow you now, please stay in front of me")
                rospy.sleep(2)
                self.following = True
                self.execute_command(Command(action="follow", complement=""))
                self.current_state = TaskManagerServer.TASK_STATES["FOLLOW_PERSON"]
            
            if self.current_state == TaskManagerServer.TASK_STATES["FOLLOW_PERSON"]:
                if self.following:
                    rospy.sleep(0.5)
                    rospy.loginfo("[INFO] Following person")
                else:
                    rospy.loginfo("[INFO] Stopped following person")
                    # command
                    self.execute_command(Command(action="stop_follow", complement=""))
                    self.current_state = TaskManagerServer.TASK_STATES["GIVE_BAG"]
                
            if self.current_state == TaskManagerServer.TASK_STATES["GIVE_BAG"]:
                rospy.loginfo("[INFO] Giving bag...")
                self.subtask_manager["hri"].speak("Please, take the bag from my gripper")
                self.execute_command(Command(action="give", complement=""))
                self.current_state = TaskManagerServer.TASK_STATES["RETURN"]
            
            if self.current_state == TaskManagerServer.TASK_STATES["RETURN"]:
                rospy.loginfo("[INFO] Returning to initial position...")
                self.subtask_manager["hri"].speak("I'm going to return to my initial position")
                self.execute_command(Command(action="go", complement="past location"))
                self.current_state = TaskManagerServer.TASK_STATES["SHUTDOWN"]
            
            if self.current_state == TaskManagerServer.TASK_STATES["SHUTDOWN"]:
                rospy.loginfo("[INFO] Returning to initial position...")
                self.subtask_manager["hri"].speak("I have finished my tasks, I'm going to rest now")
                break
            self._rate.sleep()
        rospy.loginfo("[SUCCESS] Task Finished")

    def stop_following_callback(self, msg: Bool) -> None:
        """Callback to stop following the person"""
        rospy.loginfo(f"[INFO] Received stop signal: {msg.data}")
        self.following = not msg.data

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr(f'Error: {e}')
