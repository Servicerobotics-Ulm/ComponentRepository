#!/usr/bin/python
import sys
sys.path.append('/opt/comps/examples/python/')
from openravepy import *
from numpy import *
from str2num import *
from rodrigues import *
from TransformMatrix import *
from TSR import *
import time
import os

import inspect

def PrintFrame():
  callerframerecord = inspect.stack()[1]    # 0 represents this line
                                            # 1 represents line at caller
  frame = callerframerecord[0]
  info = inspect.getframeinfo(frame)
  print "------------- TEMP. DEBUG OUTPUT --------------"
  print "in File: \t" + str(info.filename)
  print "in Function: \t" + str(info.function)
  print "in Line: \t" + str(info.lineno)
  print "------------- /TEMP. DEBUG OUTPUT --------------"

class CoMPSDoorManipulation:
    orUtil = None
    environment = None
    furnitureName = None
    furnitureRobot = None
    furnitureKinBody = None
    probs_cbirrt_furniture = None
    robot = None
    probs_cbirrt_Robot = None
    startBounds = None
    goalBounds = None
    handleBounds = None
    def __init__(self, orUtilInst, furniture):
        try:
            self.orUtil = orUtilInst
            self.furnitureName = furniture;
            self.init()
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR initializing COMPS Constraint Manipulation"
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "Constructor Parameters were:"
            print >> sys.stderr, "Instance of ORUtil  : " + str(orUtilInst)
            print >> sys.stderr, "Furniture Name      : " + str(furniture)
            print >> sys.stderr, "-----------------------------------------------------"
        
    
    def init(self):
        self.environment = self.orUtil.env
        self.furnitureRobot = self.environment.GetRobot(self.furnitureName)
        if self.furnitureRobot is not None:
            try:
                self.furnitureKinBody = self.environment.GetKinBody(self.furnitureName)
                self.robot = self.environment.GetRobot(self.orUtil.robot.GetName())
                
                self.probs_cbirrt_Robot = RaveCreateProblem(self.environment,'CBiRRT')
                self.environment.LoadProblem(self.probs_cbirrt_Robot, self.robot.GetName())
                
                self.probs_cbirrt_furniture = RaveCreateProblem(self.environment,'CBiRRT')
                self.environment.LoadProblem(self.probs_cbirrt_furniture, self.furnitureName)
            except Exception as ex:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR initializing COMPS Constraint Manipulation" + str(sys.exc_info()[0])
                print >> sys.stderr, "additional informations (if any): " +  str(ex)
                print >> sys.stderr, "-----------------------------------------------------"
        else:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR initializing COMPS Constraint Manipulation" 
            print >> sys.stderr, "additional informations (if any): " + "Furniture Robot could not be found"
            print >> sys.stderr, "Given Furniture Robot name was  : " + str(self.furnitureName)
            print >> sys.stderr, "-----------------------------------------------------"
            
        
        
    def updateState(self):
        self.init()
        
    
    def sampleIKForPose(self, transform, offset, boundaries):
        try:
            #serialize the TSR
            TSRstring1 = SerializeTSR(0,'NULL',transform,offset,boundaries)
            #Generate the TSRChain
            TSRChainString1 = SerializeTSRChain(0,1,0,1,TSRstring1,'NULL',[])
            
            probs_cbirrt_tmp = RaveCreateProblem(self.environment,'CBiRRT')
            self.environment.LoadProblem(probs_cbirrt_tmp, self.robot.GetName())
            #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
            try:
                os.remove("cmovetraj.txt")
            except Exception as ex:
                print ex
        
            if (int(probs_cbirrt_tmp.SendCommand('RunCBiRRT psample 0.5 %s'%TSRChainString1)) == 1):
                #get the resultung trajectory
                trajFile = open("cmovetraj.txt")
                trajRobot = RaveCreateTrajectory(self.environment, "")
                trajString = trajFile.read()
                trajFile.close()
                trajRobot.deserialize(trajString)
                #get the last joint value which is the goal configuration
                return trajRobot.Sample(trajRobot.GetDuration())[0:6],trajRobot
            else:
                return None,None
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR in sampleIKForPose of DoorManipulation: " + str(sys.exc_info()[0])
            print >> sys.stderr, "number of DOFs including gripper is: " + str(len(self.maskWithGripper))
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return None,None
    
    def getJointTypeOf(self, name):
        return self.furnitureKinBody.GetJoint(name + "_handle_grasp_Joint").GetType()

    def handleTransformOf(self, name):
        link = self.furnitureKinBody.GetLink(name + "_handle_grasp_Link")
        return mat(link.GetTransform())
    
    def getRobotJointIndexOf(self, name, robot):
        return robot.GetJointIndex(name + "_grasp_Joint")
    
    def detectedHandleAt(self, doorName, transform):
        body = self.furnitureKinBody
        robot = self.furnitureRobot
        handleTransform = self.handleTransformOf(doorName)
        offsetOriginToBody = linalg.inv(mat(body.GetTransform()))
        offsetBodyToHandle = handleTransform * linalg.inv(mat(body.GetTransform()))
        newBodyPose = mat(transform) * linalg.inv(offsetBodyToHandle) * linalg.inv(offsetOriginToBody)
        newBodyPose = numpy.array(newBodyPose[:][:]) #convert the matrix to an array
        body.SetTransform(newBodyPose)
        
    def getGraspOffset(self):
        offsetHandleToHand = MakeTransform(rodrigues([0, 0, pi/2])*rodrigues([pi/2, 0, 0]), mat([-0.01, 0 ,0]).T)
        #offsetHandleToHand = MakeTransform(rodrigues([0, 0, pi/2])*rodrigues([pi/2, 0, 0]), mat([-0.03 ,0 ,0]).T)
        return offsetHandleToHand
    
    def generateChain(self, offsetHandleToHand, handlePose, Bw0, Bw1, furnitureName, jointIndex, cbirrtProb):
        jointtm = str2num(cbirrtProb.SendCommand('GetJointTransform name ' + furnitureName + ' jointind %d'%jointIndex))
        T0_w0 = MakeTransform(handlePose[:3][:,0:3],mat(jointtm[9:12]).T)
        Tw1_e = offsetHandleToHand
        Tw0_e = linalg.inv(T0_w0)*handlePose
        TSRstring1 = SerializeTSR(0,'NULL',T0_w0,Tw0_e,Bw0)
        TSRstring2 = SerializeTSR(0,'NULL',mat(eye(4)),Tw1_e,Bw1)
        TSRChainString = SerializeTSRChain(0,0,1,2,TSRstring1 + ' ' + TSRstring2,furnitureName,mat(jointIndex))
        return TSRChainString
    
    def displayCoodCross(self, env, transform, name):
        body = env.ReadKinBodyXMLFile('coordAxis.kinbody.xml')
        body.SetName(name)
        env.AddKinBody(body)
        body.SetTransform(transform.tolist())
        return body
    
    #planning opening doors and drawers.
    #@param door <string> the id of the door that should be opened. This id is defined in the model of the Furniture (=openrave robot) usually counted from 1 to n in the order up->down left->right
    #@param openAmount <float> the target amount how much the door should be opened. variable in rad for doors and meters for drawers
    #@param startBounds <mat> 1x12 matrix (xLower, xHigher... xRotLower, xRotUpper...) defining the valid boundaries for the grasp pose at the door/drawer handle at the start(closed) position 
    #@param goalBounds <mat> 1x12 matrix (xLower, xHigher... xRotLower, xRotUpper...) defining the valid boundaries for the grasp pose at the door/drawer handle at the start(opened) position 
    #@param handleBounds <mat> 1x12 matrix (xLower, xHigher... xRotLower, xRotUpper...) defining the valid boundaries for the grasp pose at the door/drawer handle during opening 
    #@return <list> list of length 2 containing the trajectories. [0] for the motion to the door handle and [1] for the opening motion
    #NOTE: the boundaries must be valid in order to ensure so that the goal bounds can be reached, based on the start-bounds and handleBounds
    #NOTE: if no boundaries are specified, certain default boundaries are taken
    def openDoorPlanning(self, door, openAmount, startBounds=None, goalBounds=None, handleBounds=None):
        #self.startBounds = startBounds if startBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0,0,   -pi/8, pi/8])
        #self.goalBounds = goalBounds if goalBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/8, pi/8])
        #self.handleBounds = handleBounds if handleBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/8, pi/8])
        #self.startBounds = startBounds if startBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0,0,   -pi/32, pi/32])
        #self.goalBounds = goalBounds if goalBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/32, pi/32])
        #self.handleBounds = handleBounds if handleBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/32, pi/32])
        #self.startBounds = startBounds if startBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0,0,   -pi/32, pi/32])
        self.startBounds = startBounds if startBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0,0,   -pi/16, pi/16])
        self.goalBounds = goalBounds if goalBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/8, pi/8])
        self.handleBounds = handleBounds if handleBounds is not None else mat([0, 0,   0, 0,   0, 0,   0, 0,   0, 0,   -pi/8, pi/8])
        result = list()
        self.updateState()
        
        #!!!!! insert temporary doorHandleTransform for testing Purpose
        #!!!!! insert temporary doorHandleTransform for testing Purpose
        #self.loadEnvironment('/usr/share/openrave-0.8/robots/ikeasideBoard.robot.xml')
        #kitchenBody = self.getKinBody('sideBoard');
        
        #doorHandleRealTransform = MakeTransform(rodrigues([0,0,0]), mat([1.0,-0.1,0.4]).T)
        #self.detectedHandleAt(door, doorHandleRealTransform)
        #!!!!! insert temporary doorHandleTransform for testing Purpose
        #!!!!! insert temporary doorHandleTransform for testing Purpose
        #raw_input("du bist hier und die kueche ist geladen")
        
        #----------first initialize certain needed values--------------
        
        #this works assumed that all TCPs have the same offset to the world/robot coordinate system
        offsetRotWorldToTCP = MakeTransform(rodrigues([0, 0, pi/2])*rodrigues([pi/2, 0, 0]), mat([0,0,0]).T)
        activedofs = self.orUtil.maskWithoutGripper
        self.robot.SetActiveDOFs(activedofs)
        initRobotDofValues = self.robot.GetActiveDOFValues()
        initFurnitureDofs = self.furnitureRobot.GetActiveDOFValues()
        doorjointind = self.getRobotJointIndexOf(door, self.furnitureRobot)
        self.furnitureRobot.SetActiveManipulator('dummy') #hardcoded name of the manipulator of the furniture robot
        
        
        
        #temporarly set the opened door to the desired open angle
        origOpenAmount  = initFurnitureDofs[doorjointind]
        tmpFurnitureDOFValues = initFurnitureDofs
        tmpFurnitureDOFValues[doorjointind] = openAmount
        self.furnitureRobot.SetActiveDOFValues(tmpFurnitureDOFValues)
        #raw_input("should now be open")
        
        #sample for an IK solution at the handle in its goal pose
        goalHandleTransform = self.handleTransformOf(door)
        goalOffsetHandleToHand = self.getGraspOffset()
        
        #self.displayCoodCross(self.environment, goalHandleTransform * goalOffsetHandleToHand, "grasp_pose")
        #self.displayCoodCross(self.environment, goalHandleTransform, "handle_pose")
        #raw_input("bofore goal ik")
        #self.environment.Remove(self.environment.GetKinBody("grasp_pose"))
        #self.environment.Remove(self.environment.GetKinBody("handle_pose"))
        goalIK,dummyObject = self.sampleIKForPose(goalHandleTransform, goalOffsetHandleToHand, self.goalBounds)
        if goalIK is None or dummyObject is None:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to perform Door Manipulation "
            print >> sys.stderr, "additional informations (if any): unable to find IK-solution for the goal configuration (= configuration for the opened door)"
            print >> sys.stderr, "please note: this message doesn't state anything about the start configuration as the goal configuration is determined beforehand"
            print >> sys.stderr, "goal handle transform were: "
            print >> sys.stderr, str(goalHandleTransform)
            print >> sys.stderr, "boundaries were: " 
            print >> sys.stderr, str(self.goalBounds)
            print >> sys.stderr, "-----------------------------------------------------";
            return list();
        print "GoalIK (openDoor)" + str(goalIK)
        #set the manipulator DOF values to the found goalIK in order the get the pose the the sampled IK 
        self.robot.SetActiveDOFValues(goalIK)
        #raw_input("this is the goalIK")
        resultingGoalTCP_Pose = mat(self.robot.GetActiveManipulator().GetEndEffectorTransform())
        #calculate the actual offset from the handle to the TCP pose that was sampled beforehand
        offsetToHandleAfterIK = linalg.inv(resultingGoalTCP_Pose) * goalHandleTransform
        
        #set the robot and the furniture back to the initial DOF values which are the actual start position for planning the path
        self.robot.SetActiveDOFValues(initRobotDofValues)
        initFurnitureDofs[doorjointind] = origOpenAmount
        self.furnitureRobot.SetActiveDOFValues(initFurnitureDofs)
        
        #calculate the actual start pose of the tcp
        startHandlePose = self.handleTransformOf(door)
        graspPose_start = startHandlePose * linalg.inv(offsetToHandleAfterIK)
        startOffsetHandleToHand = self.getGraspOffset()
        
        
        #self.displayCoodCross(self.environment, startHandlePose * startOffsetHandleToHand, "grasp_pose")
        #self.displayCoodCross(self.environment, startHandlePose, "handle_pose")
        #raw_input("before start IK")
        #self.environment.Remove(self.environment.GetKinBody("grasp_pose"))
        #self.environment.Remove(self.environment.GetKinBody("handle_pose"))
        
        #sample for a startIK
        startIK, toHandleTraj = self.sampleIKForPose(startHandlePose, startOffsetHandleToHand, self.startBounds)
        if startIK is None or toHandleTraj is None:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to perform Door Manipulation "
            print >> sys.stderr, "additional informations (if any): unable to find IK-Solution for the start configuration (= configuration for the initial door pose)"
            print >> sys.stderr, "start handle transform were: " 
            print >> sys.stderr, str(startHandlePose)
            print >> sys.stderr, "boundaries were: "
            print >> sys.stderr, str(self.startBounds)
            print >> sys.stderr, "-----------------------------------------------------";
            return list();
        print "StartIK (closedDoor)" + str(startIK)
        result.append(toHandleTraj)
        self.robot.SetActiveDOFValues(startIK)
        #raw_input("this is the startIK")
        
        
        #set the boundaries about the tcp<->handle joint according to the type of joint (currently hardcoded limits)
        if str(self.getJointTypeOf(door)) == "Prismatic":
            Bw0 = mat([-1, 1,   0, 0,   0, 0,   0, 0,  0, 0,   0, 0])
        else:
            Bw0 = mat([0, 0,   0, 0,   0, 0,   0, 0,  0, 0,   -pi, pi])
        
        #generate the TSR-chain
        tsr_chain = self.generateChain(linalg.inv(offsetToHandleAfterIK), self.handleTransformOf(door), Bw0, self.handleBounds, self.furnitureName, doorjointind, self.probs_cbirrt_furniture)
        
        #keep track of how many DOF of the mimic body are being mimiced
        numTSRChainMimicDOF = 1;
        

        #set robot and furniture to starting configuration
#        self.robot.SetActiveDOFValues(startIK)
#        self.furnitureRobot.SetActiveDOFValues(initFurnitureDofs)
#        self.furnitureRobot.SetActiveDOFs([doorjointind]) #only set the manipulated joint active
        mimicvals = mat(zeros([1,numTSRChainMimicDOF]))
        goalik = mat(goalIK)
        goaljoints = bmat('goalik  mimicvals')
        
        self.furnitureRobot.SetActiveDOFValues(initFurnitureDofs)
        self.furnitureRobot.SetActiveDOFs([doorjointind]) #only set the manipulated joint active
        self.robot.SetActiveDOFValues(startIK)
#        
#        tmpFurnitureDOFValues[doorjointind] = openAmount
#        self.furnitureRobot.SetActiveDOFValues(tmpFurnitureDOFValues)
#        self.furnitureRobot.SetActiveDOFs([doorjointind]) #only set the manipulated joint active
#        self.robot.SetActiveDOFValues(startIK)
#        raw_input("this is the goal IK")
        
        
        #raw_input("ready to plan")
        #call the cbirrt planner, it will generate a file with the trajectory called 'cmovetraj.txt'
        try:
            print "Problem Goal Joints: " + str(goaljoints)
            print self.probs_cbirrt_Robot.SendCommand('RunCBiRRT jointgoals %d %s %s'%(size(goaljoints),Serialize1DMatrix(goaljoints), tsr_chain))
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to perform Door Manipulation "
            print >> sys.stderr, "was unable to plan a path for door opening " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------" 
            return list();
        #set the door and the robot to its initial position as they were before planning
        self.furnitureRobot.SetActiveDOFValues(initFurnitureDofs)
        self.robot.SetActiveDOFValues(initRobotDofValues)
        
        try:
            probs_cbirrt_tmp = RaveCreateProblem(self.environment,'CBiRRT')
            self.environment.LoadProblem(probs_cbirrt_tmp,self.furnitureName)
            trajFile = open("cmovetraj.txt")
            trajRobot = RaveCreateTrajectory(self.environment, "")
            trajString = trajFile.read()
            trajFile.close()
            trajRobot.deserialize(trajString)
            result.append(trajRobot)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to perform Door Manipulation "
            print >> sys.stderr, "was unable to create trajectory from planned path" + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------" 
            return list();
        return result
    
    def performOpenDoorAnimation(self, trajRobot, trajDoor):
        try:
            numDOFs = len(self.orUtil.maskWithoutGripper)
            
            #Allways get the elements in the trajectory from the manipulator
            #not those from the kitchen!!!
            confManip = self.robot.GetActiveConfigurationSpecification()
            confManip.AddGroup(self.robot.GetName(),numDOFs,"")

            fValues = self.furnitureRobot.GetActiveDOFValues()
            
            confKitchen = self.furnitureRobot.GetActiveConfigurationSpecification()
            confKitchen.AddGroup(self.furnitureRobot.GetName(),len(fValues),"")

            if(self.orUtil.waitForAnimation):
                self.robot.GetController().SetPath(trajRobot)
                self.robot.WaitForController(0)
                self.robot.GetController().Reset(0)
                #temporarily just set the final position for the furniture because somehow the animation using the correct trajectory doesn't work
                #self.furnitureRobot.SetActiveDOFValues([trajDoor.Sample(trajDoor.GetDuration())[len(self.orUtil.maskWithGripper)-1]])
                self.furnitureRobot.SetActiveDOFValues([trajDoor.Sample(trajDoor.GetDuration(),confKitchen)[0:len(fValues)]])
            else:
                self.robot.SetActiveDOFValues(trajRobot.Sample(trajRobot.GetDuration(),confManip)[0:len(self.orUtil.maskWithoutGripper)])
                self.furnitureRobot.SetActiveDOFValues([trajDoor.Sample(trajDoor.GetDuration(),confKitchen)[0:len(fValues)]])
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to perform door-opening animation " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------" 
        
        
