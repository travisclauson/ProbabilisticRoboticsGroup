import pybullet as p
import time
import pybullet_data

# URDF locations - I prefer either jaco  or panda (9)
movo = "../pybullet-planning/models/movo_description/movo.urdf"
jaco = "../pybullet-planning/models/drake/jaco_description/urdf/j2n6s300.urdf"
panda = "../pybullet-planning/models/franka_description/robots/panda_arm_hand.urdf"
kinect = "../pybullet-planning/models/kinect/kinect.urdf"
pan = "../pybullet-planning/models/dinnerware/pan_tefal.urdf"
block_dir = "../pybullet-planning/models/drake/objects/block_for_pick_and_place.urdf"
ball = "sphere_with_restitution.urdf"

# SET UP ENVIRONMENT
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
useRealTime = 0
p.setRealTimeSimulation(useRealTime)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

# SET UP ROBOT
startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF(panda, startPos, startOrientation, useFixedBase = 1)
object = p.loadURDF(ball, [0.5,0.5,0.5], startOrientation)

# USER ADJUSTABLE PARAMETERS ------------------------------
x_Pos = p.addUserDebugParameter("x_pos", -1,1,0)
y_Pos = p.addUserDebugParameter("y_pos", -1,1,0)
z_Pos = p.addUserDebugParameter("z_pos", 0,2,0)

x_Orn = p.addUserDebugParameter("x_orn", 0,3.14,0)
y_Orn = p.addUserDebugParameter("y_orn", 0,3.14,0)
z_Orn = p.addUserDebugParameter("z_orn", 0,3.14,0)

gripper1 = p.addUserDebugParameter("grip1", 0,0.04,0.04)
gripper2 = p.addUserDebugParameter("grip2", 0,0.04,0.04)
# ---------------------------------------------------------

numJoints = 9

while(1):

    if(useRealTime ==0):
        p.stepSimulation()
    time.sleep(1./100.)

    # READ ALL THE USER ADJUSTABLE VALUES ----------------
    x_targetPos = p.readUserDebugParameter(x_Pos)
    y_targetPos = p.readUserDebugParameter(y_Pos)
    z_targetPos = p.readUserDebugParameter(z_Pos)
    targetPos = [x_targetPos, y_targetPos, y_targetPos]

    x_targetOrn = p.readUserDebugParameter(x_Orn)
    y_targetOrn = p.readUserDebugParameter(y_Orn)
    z_targetOrn = p.readUserDebugParameter(z_Orn)
    targetOrn = [x_targetOrn, y_targetOrn, z_targetOrn]

    grip1_targetPos = p.readUserDebugParameter(gripper1)
    grip2_targetPos = p.readUserDebugParameter(gripper2)
    #-----------------------------------------------------

    # track the position of the object for the robot to follow
    cubePos, cubeOrn = p.getBasePositionAndOrientation(object)

    # for target position: use cubePos for ball tracking / use targetPos for user adjustable 
    targetOrientation = p.getQuaternionFromEuler([0,0,0])
    targetPosJoints = p.calculateInverseKinematics(robot, numJoints, cubePos, targetOrientation=targetOrientation)  
    p.setJointMotorControlArray(robot, range(numJoints), p.POSITION_CONTROL, targetPositions=targetPosJoints)
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, grip1_targetPos)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, grip2_targetPos)

p.disconnect()
