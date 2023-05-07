import pybullet as p
import time
import pybullet_data
from pyb_utils.collision import NamedCollisionObject, CollisionDetector

# URDF locations - I prefer either jaco or panda (9 DOF)
movo = "../COMP141-FinalProject/pybullet-planning/models/movo_description/movo.urdf"
jaco = "../COMP141-FinalProject/pybullet-planning/models/drake/jaco_description/urdf/j2n6s300.urdf"
panda = "/home/selinaspry/Documents/COMP141-Probabilistic_Robotics/COMP141-FinalProject/ProbabilisticRoboticsGroup/CS141 Final Project/Panda_Juggling/resources/panda_arm_hand.urdf"
kinect = "../COMP141-FinalProject/pybullet-planning/models/kinect/kinect.urdf"
pan = "../COMP141-FinalProject/pybullet-planning/models/dinnerware/pan_tefal.urdf"
block = "../COMP141-FinalProject/pybullet-planning/models/drake/objects/block_for_pick_and_place.urdf"
ball = "sphere_small.urdf"
bouncy_ball = "/home/selinaspry/Documents/COMP141-Probabilistic_Robotics/COMP141-FinalProject/ProbabilisticRoboticsGroup/CS141 Final Project/Panda_Juggling/resources/bouncy_ball.urdf"
numJoints = 9

# SET UP ENVIRONMENT
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
col_id = p.connect(p.DIRECT)
# collision_bodies = load_enviorn
useRealTime = 0
p.setRealTimeSimulation(useRealTime)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
p.setPhysicsEngineParameter(restitutionVelocityThreshold=0)
plane = p.loadURDF("plane.urdf")

# SET UP ROBOT
robot_startPos = [0,0,0]
object_startPos = [0,0,12] #[0.4,0.6,0.6]
paddle_startPos = [0,0,1.2]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robot = p.loadURDF(panda, robot_startPos, startOrientation, useFixedBase = 1)
object = p.loadURDF(ball, object_startPos, startOrientation)
# paddle = p.loadURDF(pan, paddle_startPos, startOrientation)

# SET UP SO BOUNCING IS ENABLED --> restitution of 1 might be too much
p.changeDynamics(object, -1, restitution=1.)
p.changeDynamics(plane, -1, restitution=1.)
# p.changeDynamics(paddle, -1, restitution=1.)

# CONSTRAINT EXAMPLE
# https://www.programcreek.com/python/example/122104/pybullet.createConstraint
# p.resetJointState(paddle,panda_leftfinger,0)
# p.resetJointState(paddle,panda_rightfinger,0)
test_ore = p.getQuaternionFromEuler([0,0,0])
# p.createConstraint(parentBodyUniqueId=p.getBodyUniqueId(robot),
#                     parentLinkIndex=7,
#                     childBodyUniqueId=p.getBodyUniqueId(paddle),
#                     childLinkIndex=-1,
#                     jointType=p.JOINT_FIXED,
#                     jointAxis=startOrientation,
#                     parentFramePosition=robot_startPos,
#                     childFramePosition=robot_startPos,
#                     parentFrameOrientation=test_ore,
#                     childFrameOrientation=test_ore)

# USER ADJUSTABLE PARAMETERS ------------------------------
x_Pos = p.addUserDebugParameter("x_pos", -1,1,0)
y_Pos = p.addUserDebugParameter("y_pos", -1,1,0)
z_Pos = p.addUserDebugParameter("z_pos", 0,2,0)

x_Orn = p.addUserDebugParameter("x_orn", -3.14,3.14,0)
y_Orn = p.addUserDebugParameter("y_orn", -3.14,3.14,0)
z_Orn = p.addUserDebugParameter("z_orn", -3.14,3.14,0)

gripper1 = p.addUserDebugParameter("grip1", 0,0.04,0.04)
gripper2 = p.addUserDebugParameter("grip2", 0,0.04,0.04)
# ---------------------------------------------------------
i=1

ball_CO = NamedCollisionObject(bouncy_ball)
robot_CO = NamedCollisionObject(robot)
reward = 0
contact_count = 0
# col_id

# col_detector = CollisionDetector(
#     robot,  # client ID for collision physics server
#     collision_bodies,  # bodies in the simulation
#     # these are the pairs of objects to compute distances between
#     [(bouncy_ball, robot)],
# )

while(1):

    if(useRealTime ==0):
        p.stepSimulation()
    time.sleep(1./100.)

    # if i == 1:

    # READ ALL THE USER ADJUSTABLE VALUES ----------------
    x_targetPos = p.readUserDebugParameter(x_Pos)
    y_targetPos = p.readUserDebugParameter(y_Pos)
    z_targetPos = p.readUserDebugParameter(z_Pos)
    targetPos = [1, 1, 1]

    x_targetOrn = p.readUserDebugParameter(x_Orn)
    y_targetOrn = p.readUserDebugParameter(y_Orn)
    z_targetOrn = p.readUserDebugParameter(z_Orn)
    targetOrn = [x_targetOrn, y_targetOrn, z_targetOrn]

    grip1_targetPos = p.readUserDebugParameter(gripper1)
    grip2_targetPos = p.readUserDebugParameter(gripper2)
    #-----------------------------------------------------

    # track the position of the object for the robot to follow
    objectPos, objectOrn = p.getBasePositionAndOrientation(object)
    robotPos, robotOrn = p.getBasePositionAndOrientation(robot)

    # TARGET POSTIION: use objectPos for object tracking / use targetPos for user adjustable 
    targetOrientation = p.getQuaternionFromEuler([1,2,0])
    # targetPosJoints = p.calculateInverseKinematics(robot, numJoints, objectPos, targetOrientation=targetOrientation) 
    # targetPositions = [objectPos[0], objectPos[1], objectPos[2]]
    targetPositions = targetPos
    # print(targetPositions)
    targetPosJoints = p.calculateInverseKinematics(robot, numJoints, targetPositions)  
    p.setJointMotorControlArray(robot, range(numJoints), p.POSITION_CONTROL, targetPositions=targetPosJoints)
    p.setJointMotorControl2(robot, 9, p.POSITION_CONTROL, grip1_targetPos)
    p.setJointMotorControl2(robot, 10, p.POSITION_CONTROL, grip2_targetPos)

    curr_contact = p.getContactPoints(robot,object)
    if(curr_contact != ()):
        if(last_collision == False):
            reward += 1
        last_collision = True 
        print(curr_contact,"\n\n")
        contact_count +=1
    else: 
        last_collision = False
    print("Collisions: ", contact_count)
    print("Rewards: ",reward)
    # p.createConstraint(p.getBodyUniqueId(paddle), -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 1])
    
    # print(p.getLinkState(robot,9,0,1))

p.resetSimulation()
p.disconnect()
