#!/usr/bin/env python3

import rospy
import actionlib

COMMANDS_TOPIC = "/task_manager/commands"

class TaskManagerServer:
    def __init__(self):
        self._node = rospy.init_node("task_manager_server")

if __name__ == "__main__":
    try:
        TaskManagerServer()
    except rospy.ROSInterruptException as e:
        rospy.logerr("Error: {}".format(e))
        pass