# Instructions for executing tasks

## Receptionist

The task manager in charge of executing this task is `receptionist_task_manager.py`.

### HRI

The launch file `language_processing.launch` executes *almost* all the neccesary nodes for the **language proccessing** pipeline:
- `command_interpreter.py`: Receives raw text and publishes processed commands. 
- `conversation.py`: Receives instructions to interact with the user when receiving the interact or ask commands.
- `stop_lister.py`: Stops the robot when listening the keyword **stop**.
- `guest_analyzer.py`: Uses the GPT vision model to extract characteristics from the guest's face.

The launch file `speech.launch` executes the **speech** pipeline. Make sure to make the neccesary setup steps in the devices being used. (Refer to this [README](https://github.com/RoBorregos/home-hri/tree/main/ws/src/speech)).

### Vision

The launch file `receptionist.launch` executes the nodes needed:
- `FaceRecognition.py`: Find faces and returns the bounding boxes. Assign names when calling the service `/new_name`.
- `PersonDetection.py`: Checks if there is a person's body with the `/check_person`service. And includes the `/find_seat` service, it's in this node to avoid loading the Yolo model twice.

### Manipulation
`arm_joint_server`: For moving the arm to predefined locations or following a face.

### Navigation
Nodes neccesary for navigation between rooms.