#!/usr/bin/python
#--------------------------------------------------------------------------
#  Copyright (C) 2012, 2017 Timo Hegele, Timo Blender
#
#
#        Christian Schlegel (schlegel@hs-ulm.de)
#        University of Applied Sciences
#        Prittwitzstr. 10
#        89075 Ulm (Germany)
#
#  This file is part of the "SmartOpenRave component".
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#---------------------------------------------------------------------

#this is a utility class used for easy issuing openRave functionalities

import sys
sys.path.append('/opt/comps/examples/python/')
from sys import *
import numpy
from numpy import *
import openravepy
from openravepy import *
from openravepy.misc import InitOpenRAVELogging
import CoMPSDoorManipulation
from CoMPSDoorManipulation import *
import CoMPSSimpleManipulation
from CoMPSSimpleManipulation import *
import time
import inspect

#simply sorts two values and aligns them in the order they were given as parameter
#this means that after the call the larger value will always be stored in val2
def simpleSortDictValues(dictName, val1, val2):
    if(dictName[val1] > dictName[val2]):
        tmp = dictName[val2]
        dictName[val2] = dictName[val1]
        dictName[val1] = tmp
        
def displayCoodCross(env, transform, name):
    body = env.ReadKinBodyXMLFile('models/axes/coordAxis.kinbody.xml')
    body.SetName(name)
    env.AddKinBody(body)
    body.SetTransform(transform.tolist())
    return body

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

#calculates the unit vector which points from a given source to a target location
def calcUnitVector(sourceLocation, targetLocation):
    result =  targetLocation - sourceLocation
    result /= linalg.norm(result)
    return result
#                                                rotX    rotY    rotZ
def generateHomogenTransformationMatrix(x, y, z, roll, elevation, azimuth):
    try:
        # build homog transformation matrix which holds the actual target 6D-Pose
        azimuthRotMatrix = numpy.matrix([[numpy.cos(azimuth),-numpy.sin(azimuth), 0],
                                                  [numpy.sin(azimuth), numpy.cos(azimuth), 0],
                                                  [0, 0, 1]
                                                  ])
        elevationRotMatrix = numpy.matrix([[numpy.cos(elevation), 0, numpy.sin(elevation)],
                                            [0, 1, 0],
                                            [-numpy.sin(elevation), 0, numpy.cos(elevation)]
                                            ])
        rollRotMatrix = numpy.matrix([[1, 0, 0 ],
                                                [0, numpy.cos(roll), -numpy.sin(roll)],
                                                [0, numpy.sin(roll), numpy.cos(roll)]
                                                ])
        transform = (azimuthRotMatrix * elevationRotMatrix * rollRotMatrix)
        result = numpy.matrix([[transform[0,0], transform[0,1], transform[0,2], x],
                               [transform[1,0], transform[1,1], transform[1,2], y],
                               [transform[2,0], transform[2,1], transform[2,2], z],
                               [0, 0, 0, 1]
                                          ])

    except Exception as ex:
        print >> sys.stderr, "-----------------------------------------------------"
        print >> sys.stderr, "ERROR generating the HomogenTransformationMatrix"
        print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
        print >> sys.stderr, "additional informations (if any): " +  str(ex)
        print >> sys.stderr, "-----------------------------------------------------"
        return None
    return result

#class for performing manipulation tasks. Only designed for 5D manipulators yet
class OpenRaveManipulation:
    env = None                  #the openrave environment
    robot = None                #the manipulator
    manipProblem = None         #manipulationProblem instance
    ikType = None               #the type of the ikModel to be used (e.g. IkParameterization.Type.TranslationDirection5D)
    taskprob = None             #problem instance used for pathPlanning
    speedMulti = None           #speed multiplier for the manipulator movement in the openRave simulation
    environmentFile = None      #the default environment to be loaded at the start
    isViewerEnabled = None      #flag if viewer should be enabled
    planner = None              #the type of planner that should be used for path planning (e.g. "BIRRT"
    objectInGripper = None      #reference to the object that is currently grabbed
    robotURI = None             #the URI to the file that defines the robot
    manipulatorName = None      #the name of the manipulator as specified in the definition (=robot file) (e.g. 'arm') 
    waitForAnimation = None     #flag if the path animation should be shown in the viewer
    maskWithGripper = None      #indices of active manipulator joints for the used manipulator including the gripper Joints
    maskWithoutGripper = None   #indices of active manipulator joints for the used manipulator without the gripper Joints
    ikModel = None

    #constructor taking care of creating the object. Throws exceptions!
    #NOTE! Speed multipliler is currently not used
    def __init__(self, environmentFile, isViewerEnabled, robotURI, manipulatorName, planner, maskWithGripper, maskWithoutGripper, speedMulti=1, waitForAnimation=True):
        try:
            self.environmentFile = environmentFile
            self.isViewerEnabled = isViewerEnabled
            self.robotURI = robotURI
            self.manipulatorName = manipulatorName
            self.planner = planner
            self.speedMulti = speedMulti
            self.maskWithGripper = maskWithGripper
            self.maskWithoutGripper = maskWithoutGripper
            self.waitForAnimation = waitForAnimation
            
            self.init()
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR initializing Python-Part of OpenRave"
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "Constructor Parameters were:"
            print >> sys.stderr, "environment File    : " + str(environmentFile)
            print >> sys.stderr, "isViewerEnabled     : " + str(isViewerEnabled)
            print >> sys.stderr, "robotURI            : " + str(robotURI)
            print >> sys.stderr, "manipulator Name    : " + str(manipulatorName)
            print >> sys.stderr, "planner             : " + str(planner)
            print >> sys.stderr, "speedMulti          : " + str(speedMulti)
            print >> sys.stderr, "maskWithGripper     : " + str(maskWithGripper)
            print >> sys.stderr, "maskWithoutGripper  : " + str(maskWithoutGripper)
            print >> sys.stderr, "waitForAnimation    : " + str(waitForAnimation)
            print >> sys.stderr, "-----------------------------------------------------"

    def init(self):
        print ("TIMO: INIT START")
        InitOpenRAVELogging()
        # Fatal =0, Error =1, Warn =2, Info =3, Debug =4, Verbose =5, OutputMask =0xf, VerifyPlans =0x80000000
        RaveSetDebugLevel(DebugLevel.Verbose)
        self.env = Environment() # create the environment
        colchecker = RaveCreateCollisionChecker(self.env,'ode')
        self.env.SetCollisionChecker(colchecker)
        print ("TIMO: INIT AFTER SET COL")
        self.env.Reset()
        self.loadEnvironment(self.environmentFile) # load the scene
        print ("TIMO: INIT AFTER LOAD ENV")
        self.robot = self.env.ReadRobotURI(self.robotURI)
        
        if self.isViewerEnabled == True:
            self.env.SetViewer('qtcoin') # start the viewer
            print ("TIMO: INIT AFTER SET VIEWER")
        
        self.env.AddRobot(self.robot,False)
        print ("TIMO: INIT AFTER ADD ROBOT")
        self.robot.SetActiveManipulator(self.manipulatorName)#activate the arm-manipulator

        #create manipulation problem
        self.manipProblem = interfaces.BaseManipulation(self.robot, self.planner, self.speedMulti)
        self.taskprob = interfaces.TaskManipulation(self.robot)

        #create a handle for the collision callback
        self.env.GetCollisionChecker().SetCollisionOptions(0)
        self.colDisabledObjects = [dict(),dict()]

        #pose6D = self.robot.GetActiveManipulator().GetEndEffectorTransform().tolist()
        #pose6D = self.convertFromManipIKtoZAFHCoords(pose6D).tolist()
        #self.displayCoodCross(self.env, pose6D , "grasp_pose")
        print ("TIMO: INIT END")
        
    def drawTCPAxes(self, name):
        T = self.robot.GetManipulator(name).GetTransform()
    	colors = numpy.array([[1,0,0],[1,0,0],[0,1,0],[0,1,0],[0,0,1],[0,0,1]])
    	dist=1.0
    	linewidth=1
    	self.env.drawlinelist(numpy.array([T[0:3,3],T[0:3,3]+T[0:3,0]*dist,T[0:3,3],T[0:3,3]+T[0:3,1]*dist,T[0:3,3],T[0:3,3]+T[0:3,2]*dist]),linewidth,colors=colors)

    def displayCoodCross(self, env, transform, name):
        body = env.ReadKinBodyXMLFile('/usr/local/share/openrave-0.9/robots/axes/coordAxis.kinbody.xml')
        body.SetName(name)
        env.AddKinBody(body)
        body.SetTransform(transform.tolist())
        return body
        
    def getIKModel(self):
        try:
            ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.Transform6D)
            if not ikmodel.load():
                print "-----------------------------------------------------"
                print "Trying to Generate 6D-ikmodel (This can take a while) ...."
                ikmodel.autogenerate()
                print "Successfully generated 6d-IK model!"
                print "-----------------------------------------------------"
            self.ikType = "Transform6D"
        except Exception as ex:
            try:
                print "Generation aborted, unable to generate 6D-IK model!"
                print "additional informations (if any): " +  str(ex)
                print "WARNING using 6D IK failed!!"
                print "additional informations (if any): Used Manipulator doesn't support 6D Ik - Solutions"
                print "now trying to use 5D IK..."
                
                ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=IkParameterization.Type.TranslationDirection5D)
                if not ikmodel.load():
                    print "Autogenerate ikmodel (This can take a while) ...."
                    ikmodel.autogenerate()
                    print ".. .Autogenerate ikmodel finished!"
                self.ikType = "TranslationDirection5D"
                print "Successfully generated 5D IK"
                print "-----------------------------------------------------"
            except Exception as ex:
                print >> sys.stderr, "ERROR creating IKModel for the Manipulator instance"
                print >> sys.stderr, "additional informations (if any): Unable to create IK solution Model for the specified manipulator"
                print >> sys.stderr, "exact error was: " + str(ex)
                print >> sys.stderr, "robotURI to load was: " +  str(self.robotURI)
                print >> sys.stderr, "manipulator to load was: " + str(self.manipulatorName)
                print >> sys.stderr, "-----------------------------------------------------"
                return None
        return ikmodel
        
    def getEnvClone(self, options=CloningOptions.Bodies):
        envClone = None
        try:
            with self.env:
                envClone = self.env.CloneSelf(options)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR creating a clone of the original environment " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "combined Cloning options were: " + str(options)
            print >> sys.stderr, "HINT: enum values represent:"
            print >> sys.stderr, "1  -> CloningOptions.Bodies"
            print >> sys.stderr, "2  -> CloningOptions.Viewer"
            print >> sys.stderr, "4  -> CloningOptions.Simulation"
            print >> sys.stderr, "8  -> CloningOptions.RealControllers"
            print >> sys.stderr, "16 -> CloningOptions.Sensors"
            print >> sys.stderr, "-----------------------------------------------------"    
        return envClone

    def collisioncallback(self, report,fromphysics):
        print "collisionCheckn nr.: " + str(self.cnttmp) + "between " + str(report.plink1) + " and " + str(report.plink1)
        if(not self.isCollisionEnabled(report.plink1, report.plink2)):
            print "ignoring the collision"
            return CollisionAction.Ignore
        else:
            print "not ignoring the collision"
            return CollisionAction.DefaultAction

    def disableCollisionFor(self, body1, body2):
        print "disabling collision"
        self.colDisabledObjects[0][body1] = body2
        self.colDisabledObjects[1][body2] = body1
    
    def enableCollisionFor(self, body1, body2):
        print "enabling collision"
        try:
            del self.colDisabledObjects[0][body1]
            del self.colDisabledObjects[1][body2]
        except Exception as ex:
            return True
        return True
        
    def isCollisionEnabled(self, body1, body2):
        print "checking if objects are in collision"
        return (self.colDisabledObjects[0].get(body1, None) is None) and (self.colDisabledObjects[0].get(body2, None) is None)

#    def test(self):
#        print "test function"
#        #print self.robot.GetManipulator('arm').GetArmJoints()
#        #print self.robot.GetManipulator('arm').GetArmIndices()
#        print self.robot.GetManipulator('arm').GetChildLinks()
#        #print len(self.robot.GetManipulator('arm').GetChildLinks())
#        #print self.robot.GetManipulator('arm').GetGripperJoints()
#        #print self.robot.GetManipulator('arm').GetEndEffector()
#        #print len(self.robot.GetManipulator('arm').GetChildLinks())
#        #print self.env.GetKinBody('UR6Schunk').GetChain(6,7,True)[0]
#        #print self.env.GetKinBody('UR6Schunk').GetChain(6,7,True)[0].GetHierarchyChildLink()
#        #print self.env.GetKinBody('UR6Schunk').GetChain(6,7,True)[0].GetHierarchyChildLink().GetRigidlyAttachedLinks()
#
#        print "test function"
#        #time.sleep(5000)

    #saves the current environment to a specified filename
    def saveEnvironment(self, filename):
        try:
            self.env.Save(filename)
            return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR saving environment/KinBody-file: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "url of the file to save was: " + str(filename)
            print >> sys.stderr, "-----------------------------------------------------"
        return False

    #simply loads an environment file into the existing environment
    def loadEnvironment(self, environmentFile=None):
        envTmp = Environment()
        envTmp.Load(environmentFile)
        with self.env:
            with envTmp:
                bodies = self.env.GetBodies()
                for body in bodies:
                    bodyName =  body.GetName()
                    if envTmp.GetKinBody(bodyName) is not None:
                        print >> sys.stderr, "-----------------------------------------------------"
                        print self.env.GetBodies()
                        print envTmp.GetKinBody(bodyName)
                        print >> sys.stderr, "ERROR loading environment/KinBody-file into current environment"
                        print >> sys.stderr, "additional informations (if any): there is already an Object with the name \"" + body.GetName() + "\" present"
                        print >> sys.stderr, "file to load was: " + str(environmentFile)
                        print >> sys.stderr, "-----------------------------------------------------"
                        return False
        envTmp.Destroy();
        try:
            with self.env:
                self.env.Load(environmentFile)
                time.sleep(1)#give the viewer time to draw the scene
                return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR loading environment/KinBody-file into current environment: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "file to load was: " + str(environmentFile)
            print >> sys.stderr, "-----------------------------------------------------"
        return False

    #create a new kinbody from a specified string and add it to the environment
    def createKinbody(self, string):
        try:
            print ("STRING", string)
            body = self.env.ReadKinBodyData(string)
	    self.env.AddKinBody(body)
            return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR creating Kinbody: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return False

    #opens the gripper
    #@param sampleTime: the timestep-length to sample the trajectory in seconds 
    def openGripper(self, sampleTime):
        try:
            result = self.taskprob.ReleaseFingers(execute=False, outputtrajobj=True)
            print "openGripper openFinges result[1]:" + str(result[1])
            #planningutils.RetimeActiveDOFTrajectory(result[1],self.robot,False,maxvelmult=1,plannername='parabolicsmoother')
            self.performTrajectory(result[1])
            return self.sampleTrajectory(result[1], sampleTime)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR opening Gripper: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return None

    #CLOSES the gripper
    #@param sampleTime the timestep-length to sample the trajectory in seconds 
    def closeGripper(self, sampleTime):
        try:
            result = self.taskprob.CloseFingers(execute=False, outputtrajobj=True) # close fingers until collision
            #print "closeGripper CloseFinges result[1]:" + str(result[1])
            #planningutils.RetimeActiveDOFTrajectory(result[1],self.robot,False,maxvelmult=1,plannername='parabolicsmoother')
            self.performTrajectory(result[1])
            return self.sampleTrajectory(result[1], sampleTime)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR closing Gripper: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return None

    #Blender, Simple version: Just attaches the object to the TCP of the manipulator (no collision check)
    #@param objectID: string identifier for the object to be grabbed
    def getObject(self, objectID):
        try:
            tmp = self.env.GetKinBody(objectID);
            if tmp is not None:
                with self.env:
                    self.robot.Grab(tmp)
                    self.objectInGripper = tmp
                    for gripperLink in self.robot.GetManipulator(self.manipulatorName).GetChildLinks():
                        for objLink in tmp.GetLinks():
                            self.disableCollisionFor(gripperLink, objLink)
                return 0
            else:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR getting Object"
                print >> sys.stderr, "additional informations (if any): the desired object could not be found in the environment"
                print >> sys.stderr, "objectID to grab were: " + str(objectID)
                print >> sys.stderr, "-----------------------------------------------------"
                return -2
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR getting Object: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "objectID to grab were: " + str(objectID)
            print >> sys.stderr, "-----------------------------------------------------"
            return -3

    #Grabs a specified object and closes the gripper
    #object will be bound to the TCP of the manipulator
    #@param objectID: string identifier for the object to be grabbed
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    def grabObject(self, objectID, sampleTime):
        try:
            tmp = self.env.GetKinBody(objectID);
            closeTraj = None
            if tmp is not None:
                closeTraj = self.closeGripper(sampleTime) # close fingers until collision
                if closeTraj is not None:
                    with self.env:
                        self.robot.Grab(tmp)
                        self.objectInGripper = tmp
                    for gripperLink in self.robot.GetManipulator(self.manipulatorName).GetChildLinks():
                        for objLink in tmp.GetLinks():
                            self.disableCollisionFor(gripperLink, objLink)
                    return closeTraj
                else:
                    return -1
            else:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR grabbing Object"
                print >> sys.stderr, "additional informations (if any): the desired object could not be found in the environment"
                print >> sys.stderr, "objectID to grab were: " + str(objectID)
                print >> sys.stderr, "-----------------------------------------------------"
                return -2
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR grabbing Object: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "objectID to grab were: " + str(objectID)
            print >> sys.stderr, "-----------------------------------------------------"
            return -3
    
    #releases a specified object and opens the gripper
    #an eventually grabbed object will be unbound from the TCP of the manipulator
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    def releaseObject(self):
        if self.objectInGripper is not None:
            try:
                with self.env:
                    self.robot.Release(self.objectInGripper)
                for gripperLink in self.robot.GetManipulator(self.manipulatorName).GetChildLinks():
                    for objLink in self.objectInGripper.GetLinks():
                        self.enableCollisionFor(gripperLink, objLink)
                self.objectInGripper = None;
                return 0
            except Exception as ex:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR releasing Object: " + str(sys.exc_info()[0])
                print >> sys.stderr, "additional informations (if any): " +  str(ex)
                print >> sys.stderr, "objectID to grab were: " + str(self.objectInGripper)
                print >> sys.stderr, "-----------------------------------------------------"
                return -3
        else:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR releasing object"
            print >> sys.stderr, "additional informations (if any): there is no object in the gripper"
            print >> sys.stderr, "-----------------------------------------------------"
            return -2


    #releases a specified object and opens the gripper
    #an eventually grabbed object will be unbound from the TCP of the manipulator
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    def releaseGrabbedObject(self, sampleTime):
        if self.objectInGripper is not None:
            try:
                openTraj = self.openGripper(sampleTime)
                if openTraj is not None: # close fingers until collision
                    with self.env:
                        self.robot.Release(self.objectInGripper)
                    for gripperLink in self.robot.GetManipulator(self.manipulatorName).GetChildLinks():
                        for objLink in self.objectInGripper.GetLinks():
                            self.enableCollisionFor(gripperLink, objLink)
                    self.objectInGripper = None;
                    return openTraj
                else:
                    return -1
            except Exception as ex:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR releasing Object: " + str(sys.exc_info()[0])
                print >> sys.stderr, "additional informations (if any): " +  str(ex)
                print >> sys.stderr, "objectID to grab were: " + str(self.objectInGripper)
                print >> sys.stderr, "-----------------------------------------------------"
                return -3
        else:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR releasing object"
            print >> sys.stderr, "additional informations (if any): there is no object in the gripper"
            print >> sys.stderr, "-----------------------------------------------------"
            return -2

    #resets the environment
    #this means deleting all objects except the manipulator and the eventually grabbed object
    def resetAll(self):
        try:
            self.deleteAllKinbodies(False, False)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR resetting the scene -> unable to delete present kinbodies"
            print >> sys.stderr, "additional informations (if any):" +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return False
        try:
            result = self.loadEnvironment(self.environmentFile)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR resetting the Scene -> unable to load the default/initial scene"
            print >> sys.stderr, "additional informations (if any):" +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return False
        return result

    #adds a trimesh to the environment
    def addTriMesh(self,name, in_vertices,in_points):
        try:
            vertices_size = len(in_vertices)
            points_size = len(in_points)
            print >> sys.stderr, "vertices size: " + str(vertices_size)
            print >> sys.stderr, "points size: " + str(points_size)
            with self.env:
                body = RaveCreateKinBody(self.env,'')
                #vertices = array([[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0],[1.0,1.0,0.0]])
		#vertices = array([[0.0,0.0,0.0],[1.0,0.0,0.0],[0.0,1.0,0.0]])                
		#indices = array([[0,1,2],[0,1,3]])
                #trimesh = TriMesh(vertices,indices)
                trimesh = TriMesh(in_points,in_vertices)
                body.InitFromTrimesh(trimesh,draw=True)
                body.SetName(name)
                self.env.AddKinBody(body)

        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR adding triMesh"
            print >> sys.stderr, "additional informations (if any):" +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return False
        return True

    #deletes all kinbodies from an environment
    #@param withManiupulator: specifies if the manipulator should be cleared too
    #@param onlyManipulator: specifies if only the manipulator should be cleared
    def deleteAllKinbodies(self, withManipulator=False, onlyManipulator=False):
        try:
            with self.env:
                bodies = self.env.GetBodies()
                for body in bodies:
                    if(body != self.objectInGripper):
                        if ((body.IsRobot() and withManipulator) or (body.IsRobot() and onlyManipulator)) :
                            self.env.Remove(body)
                        else:
                            if ((not body.IsRobot()) and (not onlyManipulator)):
                                self.env.Remove(body)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR : " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"

    #deletes a specified kinbody from the environment
    #@param name: string identifier
    def deleteKinbody(self, name):
        body = self.getKinBody(name)
        if body is not None:
            if (body == self.objectInGripper):
                self.robot.Release(self.objectInGripper)
                self.objectInGripper = None;
                self.env.Remove(body)
            else:
                with self.env:
                    if (self.env.Remove(body) == True):
                        print "Deleted kinbody \"" + str(name) + "\" "
                    else:
                       print "Could not remove kinbody \"" + str(name)+ "\""
            return True
        else:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR deleting KinBody: " + str(name)
            print >> sys.stderr, "additional informations (if any): Specified KinBody was not found in the current scene"
            print >> sys.stderr, "KinBody name were: " + str(name)
            print >> sys.stderr, "-----------------------------------------------------"
            return False

    #moves a kinbody within the scene by the specified transformation
    #@param name: string identifier
    #@param transform: #@parama transform: transformation matrix of the desired pose
    def moveKinbody(self, name, transform):
        try:
            body = self.getKinBody(name)
            print("TRANSFORM", transform)
            body.SetTransform(transform)
            return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR moving KinBody: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "KinBody name were: " + str(name)
            print >> sys.stderr, "desired transformation was: " + str(transform)
            print >> sys.stderr, "HINT: make sure the KinBody exists in the current scene and pass a 4x4 transformation matrix"
            print >> sys.stderr, "-----------------------------------------------------"
        return False

    def renameKinbody(self, nameOld, nameNew):
        try:
            body = self.getKinBody(nameOld)
            body.SetName(nameNew)
            return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR renaming KinBody: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "KinBody name were: " + str(nameOld)
            print >> sys.stderr, "desired name was: " + str(nameNew)
            print >> sys.stderr, "-----------------------------------------------------"
        return False

    #set which links of the manipulator should be active
    #@param mask: list of link indices that specifies the active joints  
    def setActiveDOFs(self, mask):
        try:
            self.robot.SetActiveDOFs(mask);
            return True
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR setting active DOFs: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "mask were: " + str(mask)
            print >> sys.stderr, "-----------------------------------------------------"
        return False

    #sets the joints to the specified angles in the openRave simulation environment
    #@param joints: list of jointValues that specify the desired joint angles (in rad) the length of the list should match the number of active joints of the manipulator
    def setJoints(self, joints):
        if(len(joints) == len(self.maskWithoutGripper)):
            self.setActiveDOFs(self.maskWithoutGripper)
        elif(len(joints) == len(self.maskWithGripper)):
            self.setActiveDOFs(self.maskWithGripper)
        else:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to set joints "
            print >> sys.stderr, "additional informations (if any): given number of joint values doesn't match the number of required joint values for the manipulator"
            print >> sys.stderr, "given joints were: " + str(joints)
            print >> sys.stderr, "number of given joints were: " + str(len(joints))
            print >> sys.stderr, "number of manipulator DOFs including the gripper: " + str(len(self.maskWithGripper))
            print >> sys.stderr, "number of manipulator DOFs without the gripper: " + str(len(self.maskWithoutGripper))
            print >> sys.stderr, "-----------------------------------------------------";
            return -1;
        try:
            self.robot.SetActiveDOFValues(joints)
            self.env.UpdatePublishedBodies()
            return 0
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR unable to set joints: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "given joints were: " + str(joints)
            print >> sys.stderr, "HINT: remember that setting the joints requires specifying the angles of the gripper-unit too"
            print >> sys.stderr, "-----------------------------------------------------"
            return -2
        return 0
    
    def doDoorManipulation(self, furniture, door, openAmount, sampleTime=0.2, startBounds=None, goalBounds=None, handleBounds=None):
        result = list()
        
        #TODO: insert temporary for testing Purpose
        #TODO: insert temporary for testing Purpose

#        transTmp = self.env.GetKinBody('UR6Schunk').GetTransform()
#        transTmp[0][3] = 0.13
#        transTmp[1][3] = -0.07
#        transTmp[2][3] = 0.352
#        transTmp = MakeTransform(rodrigues([0,0,pi-pi/4])*transTmp[:3][:,0:3], mat(transTmp[0:3][:,3]).T)
#        transTmp = numpy.array(transTmp[:][:])
#        print transTmp
#        raw_input("asdasd")
#        body = self.env.GetKinBody("UR6Schunk")
#        body.SetTransform(transTmp)
#
#        initdofvals = [1.867+pi/4, -2.499, 2.497, 0.006, pi/2, -3.15, 0.0]
#        self.setJoints(initdofvals)
#        
#        
#        #self.loadEnvironment('/usr/share/openrave-0.8/robots/ikeasideBoard.robot.xml')
#        #kitchenBody = self.getKinBody('sideBoard');
#        self.loadEnvironment('/usr/share/openrave-0.8/robots/ikeaKitchenZAFH.robot.xml')
#        kitchenBody = self.getKinBody('ikeaKitchen');
#        kitchenBody.SetName("0")

        #TODO: insert temporary for testing Purpose
        #TODO: insert temporary for testing Purpose
        
        self.robot.SetActiveManipulator(self.manipulatorName+"Z")
        compsManipulation = CoMPSDoorManipulation(self, furniture)
        try:
              trajectories = compsManipulation.openDoorPlanning(door, openAmount, startBounds, goalBounds, handleBounds)
        except Exception as ex:
              print >> sys.stderr, "-----------------------------------------------------"
              print >> sys.stderr, "unexpected ERROR calling openDoorPlanning: " + str(sys.exc_info()[0])
              print >> sys.stderr, "additional informations (if any): " +  str(ex)
              print >> sys.stderr, "-----------------------------------------------------"
        if len(trajectories) > 1:
              #save a copy of the open-door-trajectory because after parabolic smoothing for the manipulator the values for the door are removed from the original traj
              trajDoor = RaveCreateTrajectory(self.env, '')
              trajDoor.Clone(trajectories[1], 1+2+4+8+16)
              trajString1 = self.sampleTrajectorySmooth(trajectories[0], sampleTime, False) # this is the trajectory which leads to the handle
              trajString2 = self.sampleTrajectorySmooth(trajectories[1], sampleTime, False) # this is the trajectory which opens the door
              #trajString1 = self.sampleTrajectory(trajectories[0], sampleTime, False) # this is the trajectory which leads to the handle
              #trajString2 = self.sampleTrajectory(trajectories[1], sampleTime, False) # this is the trajectory which opens the door
              self.performTrajectory(trajectories[0])
              compsManipulation.performOpenDoorAnimation(trajectories[1], trajDoor)
              result.append(trajString1)
              result.append(trajString2)
        self.robot.SetActiveManipulator(self.manipulatorName)
        return result

    #simulates a given trajectory with the manipulator
    #@param traj: OR-trajectory object defining the desired trajectory
    def performTrajectory(self, traj):
        numDOFs = len(self.maskWithoutGripper)
        conf = self.robot.GetActiveConfigurationSpecification()
        conf.AddGroup(self.robot.GetName(),numDOFs,"")
        if self.waitForAnimation:
            self.robot.GetController().SetPath(traj)
            self.robot.WaitForController(0)
            self.robot.GetController().Reset(0)
        else:
            self.setJoints(list(traj.Sample(traj.GetDuration(),conf)[0:len(self.maskWithoutGripper)]))


    #moves the joints using planning to the specified angles
    #plans a path to the specified joint angles of the manipulator
    #parameter match the parameters needed for the 'BIRRT' planner
    #@param joints: list of the goal joint angles
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    #@param maxiter: maximum steps of planner iterations before aborting the planning trial
    #@param maxtries: maximum tries to find a plan untill the planning fails
    def moveJoints(self, joints, sampleTime, maxiter=None ,maxtries=None):
        result = None
        #handle = self.env.RegisterCollisionCallback(self.collisioncallback)
        try:
            print "-----------------------------------------------------"
            print "Started planning"
            start = time.time()
            #if outputTraj in not none the trajectory will be printed. There is no possibility to set any printing options from the python interface (see BaseManipulation.py in method _moveActiveJoints)
            planningResult = self.manipProblem.MoveManipulator(goal=joints,maxiter=maxiter,maxtries=maxtries, execute=False, outputtrajobj=True)
            duration = time.time() - start
            print "Finished planning"
            print "duration: " + str(round(duration,2)) + " s"
            print "-----------------------------------------------------"
            #handle.close()
            result = self.sampleTrajectory(planningResult, sampleTime)
            print("RESULT_PLAN: ", result)
            #self.performTrajectory(planningResult)
        except Exception as ex:
            #handle = self.env.RegisterCollisionCallback(collisioncallback)
            report = CollisionReport()
            collision = self.env.CheckCollision(self.robot, report=report)
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR moving joints: " + str(sys.exc_info()[0])
            if(collision is True):
                print >> sys.stderr, "There is a collision between: " + report.plink1.GetParent().GetName() + ":" + report.plink1.GetName() + " and " + report.plink2.GetParent().GetName() + ":" + report.plink2.GetName()
            else:
                print >> sys.stderr, "There is no collision"
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "given joints were: " + str(joints)
            print >> sys.stderr, "-----------------------------------------------------"
            #handle.close()
            return None
        return result

    def planPathCOMPS(self):
        # The path was already determined in sampleIKForPose, therefore just read in the file that contains the path
        print ("PLAN PATH COMPS CALLED ")
        with open("cSampleIKtraj.txt") as f:
            content = f.readlines()

        content = content[7].split(" ")

        i = 0
        j = 0

        traj = []

        while j < len(content)-1:
            traj.append([])
            while j < 19*(i+1):
                if j < len(content)-1:
                    traj[i].append(float(content[j]))
                else:
                    break
                j = j + 1
            i = i + 1
        print("TRAJ: ", traj)    
        return traj
    
    #moves the joints using constrainedplanning to the specified angles
    #plans a path to the specified joint angles of the manipulator
    #parameter match the parameters needed for the 'BIRRT' planner
    #@param joints: list of the goal joint angles
    #@param params: parameter containing the boundaries of allowed movements of the TCP during the path
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    #@param maxiter: maximum steps of planner iterations before aborting the planning trial
    #@param maxtries: maximum tries to find a plan untill the planning fails
    def moveJointsConstrained(self, joints, params, sampleTime):
        try:
            params['heightFrom']           
            params['heightTo']
            params['sideFrom']           
            params['sideTo']
            params['depthFrom']
            params['depthTo']
            params['rollAngleFrom'] 
            params['rollAngleTo']
            params['elevationAngleFrom']
            params['elevationAngleTo']
            params['azimuthAngleFrom']
            params['azimuthAngleTo']
        except KeyError as ex:
            print >> sys.stderr, "ERROR: missing parameter in the parameter list for sampling for an IKSolution:" + str(ex)
            return None
        
        simpleSortDictValues(params, 'heightFrom', 'heightTo')
        simpleSortDictValues(params, 'depthFrom','depthTo')
        simpleSortDictValues(params, 'sideFrom','sideTo')
        simpleSortDictValues(params, 'rollAngleFrom', 'rollAngleTo')
        simpleSortDictValues(params, 'elevationAngleFrom', 'elevationAngleTo')
        simpleSortDictValues(params, 'azimuthAngleFrom', 'azimuthAngleTo')
        self.robot.SetActiveManipulator(self.manipulatorName+"Z")
        compsManip = CoMPSSimpleManipulation(self)
        resultTraj = compsManip.moveJointsConstrained(joints, params, sampleTime)
        self.robot.SetActiveManipulator(self.manipulatorName)
        if(resultTraj is not None):
            self.performTrajectory(resultTraj)
            result = self.sampleTrajectorySmooth(resultTraj, sampleTime, False)
            return result
        return None
    
    #samples an openRave Trajectory object, for the UR6-85-5-A with a given sampleTime and returns the trajectory
    #@param traj: openRave trajectory object to be sampled
    #@param sampleTime: sampleTime the timestep-length to sample the trajectory in seconds
    #this method assumes that the linear speed is low enought so that the real manipulator can complete even the unsmoothed traj
    def sampleTrajectorySmooth(self, traj, sampleTime, withGripper=True, accPartLen=0.25, decPartLen=0.25, travelPartMaxSpeedModifier=0.5):
        try:
            if (withGripper):
                numDOFs = len(self.maskWithGripper) 
            else:
                numDOFs = len(self.maskWithoutGripper)

            #Allways get the elements in the trajectory from the manipulator
            #not those from the kitchen!!!
            conf = self.robot.GetActiveConfigurationSpecification()
            conf.AddGroup(self.robot.GetName(),numDOFs,"")
            #conf = self.furnitureRobot.GetActiveConfigurationSpecification()
            #conf.AddGroup(self.furnitureRobot.GetName(),11,"")
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot)
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot, nmaxiterations>100'')
            #planningutils.RetimeActiveDOFTrajectory(traj,self.robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
            #planningutils::RetimeActiveDOFTrajectory(traj, probot, false, 1.0, "parabolictrajectoryretimer");
            #planningutils.RetimeActiveDOFTrajectory(traj,self.robot, 1.0,plannername='ParabolicTrajectoryRetimer')
            
            result = list()
            cnt = 0;
            curSampleTime = 0
            curSample = list()
            stepLength = sampleTime
            steps = math.ceil(traj.GetDuration() / stepLength) # get the number of necessary steps-1 (floor will only get the step number to the second-last step)
            if (steps <= 4):
                return self.sampleTrajectory(traj, sampleTime, withGripper)
            else:
                accPartSteps    = math.floor(steps * accPartLen)
                decPartSteps    = math.floor(steps * decPartLen) 
                travelPartSteps = steps - (accPartSteps + decPartSteps)
                
            
                #print "!!!!!!!!!!!!!!!!!!!!!!!!!!pythonTraj!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                while(cnt < steps):
                    #print "complete sample: " + str(traj.Sample(curSampleTime))
                    #print "joint values: " + str(traj.Sample(curSampleTime)[0:6])
                    #print "joint velocities: " + str(traj.Sample(curSampleTime)[7:13])
                    if(cnt < accPartSteps):
                        #for the firt step prevent zero-values
                        if (cnt == 0):
                            accModifier = (cnt+1)/accPartSteps
                        else:
                            accModifier = cnt/accPartSteps
                        curSample = list(traj.Sample(curSampleTime,conf)[0:numDOFs])
                        curSample.append(sampleTime*(1/(travelPartMaxSpeedModifier*accModifier)))
                    else:
                        if(cnt < accPartSteps + travelPartSteps):
                            curSample = list(traj.Sample(curSampleTime,conf)[0:numDOFs])
                            curSample.append(sampleTime*(1/travelPartMaxSpeedModifier))
                        else:
                            if(cnt < accPartSteps + travelPartSteps + decPartSteps):
                                #for the last step prevent zero-values
                                if (cnt == steps):
                                    decModifier = (steps-(cnt-1))/decPartSteps
                                else:
                                    decModifier = (steps-cnt)/decPartSteps
                                #if the last step is behind the actual end of the trajectory just add the planned config at the sampled end of the traj
                                if (curSampleTime > traj.GetDuration()):
                                    curSample = list(traj.Sample(traj.GetDuration(),conf)[0:numDOFs])
                                    curSample.append(sampleTime*(1/(travelPartMaxSpeedModifier*decModifier))) #TODO 2
                                else:
                                    curSample = list(traj.Sample(curSampleTime,conf)[0:numDOFs])
                                    curSample.append(sampleTime*(1/(travelPartMaxSpeedModifier*decModifier))) #TODO 2
                            
                            
                    result.append(curSample)
                    #result.append(list([stepLength, stepLength, stepLength, stepLength, stepLength, stepLength]))
                    #result.append(list(traj.Sample(curSampleTime)[7:13]))
                    cnt = cnt + 1
                    curSampleTime = curSampleTime + stepLength
                #print "sampleTime :" + str(traj.GetDuration() - (steps * sampleTime))
                #print curSample
                #print "!!!!!!!!!!!!!!!!!!!!!!!!!!pythonTraj!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                return result
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR sampling trajectory smoothly: " + str(sys.exc_info()[0])
            print >> sys.stderr, "number of DOFs including gripper is: " + str(len(self.maskWithGripper))
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "trajectory description were: " + str(traj)
            print >> sys.stderr, "-----------------------------------------------------"
            return None
    
    #samples an openRave Trajectory object, for the UR6-85-5-A with a given sampleTime and returns the trajectory
    #@param traj: openRave trajectory object to be sampled
    #@param sampleTime: sampleTime the timestep-length to sample the trajectory in seconds
    def sampleTrajectory(self, traj, sampleTime, withGripper=True):
        try:
            if (withGripper):
                numDOFs = len(self.maskWithGripper) 
            else:
                numDOFs = len(self.maskWithoutGripper)

            #Allways get the elements in the trajectory from the manipulator
            #not those from the kitchen!!!
            conf = self.robot.GetActiveConfigurationSpecification()
            conf.AddGroup(self.robot.GetName(),numDOFs,"")
            #conf = self.furnitureRobot.GetActiveConfigurationSpecification()
            #conf.AddGroup(self.furnitureRobot.GetName(),11,"")
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot)
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot, nmaxiterations>100'')
            #planningutils.RetimeActiveDOFTrajectory(traj,self.robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
            #planningutils.SmoothActiveDOFTrajectory(traj, self.robot,hastimestamps=False,maxvelmult=1,maxaccelmult=1,plannername='ParabolicTrajectoryRetimer')
            #planningutils::RetimeActiveDOFTrajectory(traj, probot, false, 1.0, "parabolictrajectoryretimer");
            #planningutils.RetimeActiveDOFTrajectory(traj,self.robot, 1.0,plannername='ParabolicTrajectoryRetimer')
            
            result = list()
            cnt = 0;
            curSampleTime = 0
            curSample = list()
            stepLength = sampleTime
            steps = math.ceil(traj.GetDuration() / stepLength) # get the number of necessary steps-1 (floor will only get the step number to the second-last step)
            #print "!!!!!!!!!!!!!!!!!!!!!!!!!!pythonTraj!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            while(cnt < steps):
                #print "complete sample: " + str(traj.Sample(curSampleTime))
                #print "joint values: " + str(traj.Sample(curSampleTime)[0:6])
                #print "joint velocities: " + str(traj.Sample(curSampleTime)[7:13])
                if (curSampleTime > traj.GetDuration()):
                    curSample = list(traj.Sample(traj.GetDuration(),conf)[0:numDOFs])
                    curSample.append(sampleTime*2) #TODO 2
                else:
                    curSample = list(traj.Sample(curSampleTime,conf)[0:numDOFs])
                    curSample.append(sampleTime*2) #TODO 2
                result.append(curSample)
                #result.append(list([stepLength, stepLength, stepLength, stepLength, stepLength, stepLength]))
                #result.append(list(traj.Sample(curSampleTime)[7:13]))
                cnt = cnt + 1
                curSampleTime = curSampleTime + stepLength
            #print "sampleTime :" + str(traj.GetDuration() - (steps * sampleTime))
            #print curSample
            #print "!!!!!!!!!!!!!!!!!!!!!!!!!!pythonTraj!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            return result
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR sampling trajectory: " + str(sys.exc_info()[0])
            print >> sys.stderr, "number of DOFs including gripper is: " + str(len(self.maskWithGripper))
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "trajectory description were: " + str(traj)
            print >> sys.stderr, "-----------------------------------------------------"
            return None

    #returns an openRave kinBody of an object in the scene by its name in the scene
    #@param name: stinr identifier
    def getKinBody(self, name):
        body = None
        try:
            with self.env:
                body = self.env.GetKinBody(name)
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR getting KinBody: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "body name were " + str(name)
            print >> sys.stderr, "-----------------------------------------------------"
        return body

    #returns the quat-transformation of an object in the scene by its name in the scene
    #@param name: stinr identifier
    def getKinBodyTransform(self, name):
        bodyTransform = None
        try:
            with self.env:
                bodyTransform = self.env.GetKinBody(name).GetTransform()
                return bodyTransform.tolist()
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR getting KinBodyTransform: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "body name were " + str(name)
            print >> sys.stderr, "-----------------------------------------------------"
        return bodyTransform

    #returns the 3d position (x,y,z) of an object in the scene by its name in the scene
    #@param name: stinr identifier
    def getKinBodyPos(self, name):
        pos3D = None
        try:
            with self.env:
                pos3D = self.env.GetKinBody(name).GetTransform()[0:3,3]
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR getting KinBody 3D Position: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "body name were " + str(name)
            print >> sys.stderr, "-----------------------------------------------------"
        return pos3D

#plans a path and moves the arms tool center to the position that was determined by iteration
#    !!!! parameters only fit to RRT planners !!!
#    params must include:
#    maxiter          tells the BiRRT planner how much iterations per planning procedure it should use
#    maxtries         tells the BiRRT planner how often it should start a new planning approach if the previous failed within the specified iterations
#    targetLoc3D      the target location in 3D coords (x,y,z) where the end effector of the manipulator should be moved to
#    depthFrom        offset to the target location x direction where the iteration should start
#    depthTo          offset to the target location x where the iteration should end
#    heightFrom       " same for z direction
#    heightTo         ...
#    angleFrom        same as above, angles in degree
#    angleTo          same as above, angles in degree
#    depthStepSize    the desired step size per iteration step    in centimeters
#    heightStepSize   ...                                         in centimeters
#    angleStepSize    ...                                         in angles
    def moveToolCenterByIterationTo(self, params):
        try:
            #pop the parameters that are only used within this method and !MUST! not appear in other methods (namely iterForIKSolutions)
            maxtries = params['maxtries']
            maxiter = params['maxiter']
        except KeyError as ex:
            print >> sys.stderr, "ERROR: missing parameter in the parameter list :" + str(ex)
            return "ERROR: missing parameter in the parameter list :" + str(ex)


        jointAngles = self.iterForIKSolutionsCOMPS(params)[0]
        if jointAngles is not None:
            #-----Parameters-----
            #plannername    name of the planner that is to be used none = std = "birrt"
            #maxvelmult     multiplicate velocity value fot faster animation of robot arm in environment, none = std = 1

            trajectory = self.moveJoints(jointAngles, maxiter, maxtries)
            return trajectory

    def convertFromManipIKtoZAFHCoords(self, pose6d):
            rotMat1 = numpy.matrix([
                            [1,  0,  0,  0],
                            [0,  0,  1,  0],
                            [0, -1,  0,  0],
                            [0,  0,  0,  1]
                            ])
            return  pose6d * rotMat1;

    #searches an IK-Solution for a specified TCP transformation (pose)
    #@param transform: transformation matrix of the desired pose
    def searchIKSolution(self,transform):
	print("transform", transform)
        #handle = self.env.RegisterCollisionCallback(self.collisioncallback)
        if(self.ikModel is None):
            self.ikModel = self.getIKModel()
            ikmodel = self.ikModel
        else:
            ikmodel = self.ikModel
        if (self.ikType == "Transform6D"):
            try:
                with self.env:
                    transform = numpy.mat(transform)
                    #although the variable is called UR6Offset the value represents the rotational offset to turn from OR-World-Coordinates to a armZ manipulator's TCP (which should be the same for all 6D manipulators)
                    UR6Offset = generateHomogenTransformationMatrix(0,0,0, pi/2 , 0 , pi/2 )
                    transform = transform * UR6Offset
                    
                    
                    transform = numpy.array(transform[:][:])
                    para = IkParameterization(transform,IkParameterization.Type.Transform6D)
                    solutions = ikmodel.manip.FindIKSolutions(para,IkFilterOptions.CheckEnvCollisions)
                    if solutions is not None and len(solutions) > 0:
                        solution = solutions[0]
                        #handle.close()
                        return solution
                    else:
                        return -3;
            except Exception as ex:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR searching IK solution by 6D Transform"
                print >> sys.stderr, "additional informations (if any): " +  str(ex)
                print >> sys.stderr, "given transform were " + str(transform)
                print >> sys.stderr, "-----------------------------------------------------"
                #handle.close()
                return -2
        elif (self.ikType == "TranslationDirection5D"):
            try:
                with self.env:
                    planningTarget = [transform[0][3], transform[1][3], transform[2][3]]
                    tempTransform = copy.copy(transform)
                    tempTransform[0][3] = tempTransform[1][3] = tempTransform[2][3] = 0
                    #raw_input("okok")
                    direction = numpy.matrix([[0],[0],[-1],[1]])
                    direction = tempTransform * direction
                    
                    tmpa =  direction.tolist()
                    direction = array([tmpa[0][0],tmpa[1][0],tmpa[2][0]])
                    
                    #h=self.env.drawlinestrip(array([planningTarget,planningTarget+0.1*direction]),10)
                    #time.sleep(0.2);

                    #direction = [transform[0][1], transform[1][1], transform[2][1]]
                    para = IkParameterization(Ray(planningTarget,direction),IkParameterization.Type.TranslationDirection5D)
                    solutions = ikmodel.manip.FindIKSolutions(para,IkFilterOptions.CheckEnvCollisions)
                    if solutions is not None and len(solutions) > 0:
                        #print "NUM SOLUTIONS: " + str(len(solutions))
                        solution = solutions[0]
                        #handle.close()
                        return solution
                    else:
                        return -3
            except Exception as ex:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR searching IK solution by 5D Transform"
                print >> sys.stderr, "additional informations (if any): " +  str(ex)
                print >> sys.stderr, "given transform were " + str(transform)
                print >> sys.stderr, "-----------------------------------------------------"
                #handle.close()
                return -2
        else:
                print >> sys.stderr, "-----------------------------------------------------"
                print >> sys.stderr, "ERROR generating IK solutions"
                print >> sys.stderr, "additional informations (if any): the loaded manipulator does not support 6D or 5D IK solutions"
                print >> sys.stderr, "supported and loaded IK model type is: " + self.ikType
                print >> sys.stderr, "robotURI to load was: " +  str(self.robotURI)
                print >> sys.stderr, "manipulator to load was: " + str(self.manipulatorName)
                print >> sys.stderr, "-----------------------------------------------------"
                #handle.close()
                return -1

    def iterForIKSolutionsCOMPS(self, params):
        #check if all necessary parameters are specified
        try:
            #params['roll']
            #params['elevation']
            #params['azimuth']
            params['heightFrom']           
            params['heightTo']
            params['depthFrom']
            params['depthTo']
            params['sideFrom']
            params['sideTo']
            params['targetLoc3D']
            params['rollAngleFrom'] = -4.0 
            params['rollAngleTo'] = 4.0
            params['elevationAngleFrom'] = -4.0
            params['elevationAngleTo'] = 4.0
            params['azimuthAngleFrom'] = -4.0
            params['azimuthAngleTo'] = 4.0
        except KeyError as ex:
            print >> sys.stderr, "ERROR: missing parameter in the parameter list for sampling for an IKSolution:" + str(ex)
            return None, None
        
        #now sort the values as CoMPS sampling can't handle the bounadries properly if the values are not aligned correctly
        simpleSortDictValues(params, 'heightFrom', 'heightTo')
        simpleSortDictValues(params, 'depthFrom','depthTo')
        simpleSortDictValues(params, 'sideFrom','sideTo')
        simpleSortDictValues(params, 'rollAngleFrom', 'rollAngleTo')
        simpleSortDictValues(params, 'elevationAngleFrom', 'elevationAngleTo')
        simpleSortDictValues(params, 'azimuthAngleFrom', 'azimuthAngleTo')
        
		
        
        self.robot.SetActiveManipulator(self.manipulatorName+"Z")
        compsManip = CoMPSSimpleManipulation(self)
	print("Input: ", params)
        result = compsManip.sampleIKForPose(params)
	print("Result IK: ", result)
        self.robot.SetActiveManipulator(self.manipulatorName)
        return result
        
    #just as iterForIKSolution but with a centered iteration strategy.
    def iterForIKSolutionsCentered(self, params):
        try:
            targetLoc3D            = params['targetLoc3D']
            elevation              = params['elevation']
            roll                   = params['roll']
            azimuth                = params['azimuth']

            depthFrom              = params['depthFrom']
            depthTo                = params['depthTo']
            heightFrom             = params['heightFrom']
            heightTo               = params['heightTo']
            elevationAngleFrom     = params['elevationAngleFrom']
            elevationAngleTo       = params['elevationAngleTo']
            rollAngleFrom          = params['rollAngleFrom']
            rollAngleTo            = params['rollAngleTo']
            azimuthAngleFrom       = params['azimuthAngleFrom']
            azimuthAngleTo         = params['azimuthAngleTo']

            #define step sizes
            depthStepSize           = params['depthStepSize']           # +- meter
            heightStepSize          = params['heightStepSize']          # +- meter
            elevationAngleStepSize  = params['elevationAngleStepSize']  # +- degree
            rollAngleStepSize       = params['rollAngleStepSize']       # +- degree
            azimuthAngleStepSize     = params['azimuthAngleStepSize']       # +- degree
            verbose                 = params['verbose']
        except KeyError as ex:
            print >> sys.stderr, "ERROR: missing parameter in the parameter list :" + str(ex)
            return None, None

        for key in params:
            if params[key] is None:
                if not ((key == 'maxtries') or (key == 'maxiter')):
                    print >> sys.stderr, "Parameter: " + str(key) +" must not be unspecified"
                    return None, None

#----------------------------- INIT VALUES ------------------------------------------
        try:
            tcpAxisBody = self.env.ReadKinBodyURI("models/axes/coordAxis.kinbody.xml")
            tcpAxisBody.SetName("coordCross")
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR importing coordCross"
            print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return None, None
            

        planningTarget = list(targetLoc3D) #make a copy of the original target

        depthUpperBound = max(depthFrom, depthTo)
        depthLowerBound = min(depthFrom, depthTo)
        depthUpModificator = 0
        depthDownModificator = 0

        heightUpperBound = max(heightFrom, heightTo)
        heightLowerBound = min(heightFrom, heightTo)
        heightUpModificator = 0
        heightDownModificator = 0

        elevationUpperBound = max(elevationAngleFrom, elevationAngleTo)
        elevationLowerBound = min(elevationAngleFrom, elevationAngleTo)
        elevationUpModificator = 0
        elevationDownModificator = 0

        rollUpperBound = max(rollAngleFrom, rollAngleTo)
        rollLowerBound = min(rollAngleFrom, rollAngleTo)
        rollUpModificator = 0
        rollDownModificator = 0

        azimuthUpperBound = max(azimuthAngleFrom, azimuthAngleTo)
        azimuthLowerBound = min(azimuthAngleFrom, azimuthAngleTo)
        azimuthUpModificator = 0
        azimuthDownModificator = 0

        elevationAngle  = elevation
        rollAngle       = roll
        azimuthAngle    = azimuth

        depthOffset = 0
        heightOffset = 0
        elevationAngleOffset  = 0
        rollAngleOffset  = 0
        azimuthAngleOffset = 0


        try:
            #generateHomogenTransformationMatrix(x, y, z, yaw, pitch, roll):
           #targetTransform = generateHomogenTransformationMatrix(targetLoc3D[0],targetLoc3D[1],targetLoc3D[2], numpy.deg2rad(azimuth-90), numpy.deg2rad(elevation-90), numpy.deg2rad(roll))
           targetTransform = generateHomogenTransformationMatrix(targetLoc3D[0],targetLoc3D[1],targetLoc3D[2],  numpy.deg2rad(roll), numpy.deg2rad(elevation), numpy.deg2rad(azimuth))
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR generating generateHomogenTransformation Matrix"
            print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"

        depthSwitch = 0
        heightSwitch = 0
        elevationSwitch = 0
        rollSwitch = 0
        azimuthSwitch = 0

        currentJoints = self.robot.GetActiveDOFValues()
        
        #just for displaying the status--------------
        print "-----------------------------------------------------"
        print "Iteration for a suitable IK-Solution started in Centered-Mode"
        startTime = time.time()
        nSteps =  ((depthUpperBound - depthLowerBound) / depthStepSize)                     if (((depthUpperBound - depthLowerBound) / depthStepSize) > 0)                   else 1           
        nSteps *= ((heightUpperBound - heightLowerBound) / heightStepSize)                  if (((heightUpperBound - heightLowerBound) / heightStepSize) > 0)                else 1
        nSteps *= ((elevationUpperBound - elevationLowerBound) / elevationAngleStepSize)    if (((elevationUpperBound - elevationLowerBound) / elevationAngleStepSize) > 0)  else 1
        nSteps *= ((rollUpperBound - rollLowerBound) / rollAngleStepSize)                   if (((rollUpperBound - rollLowerBound) / rollAngleStepSize) > 0)                 else 1
        nSteps *= ((azimuthUpperBound - azimuthLowerBound) / azimuthAngleStepSize)          if (((azimuthUpperBound - azimuthLowerBound) / azimuthAngleStepSize) > 0)        else 1
        nSteps += 0.3 * nSteps #TODO!!! just add an approximation of additional steps that are tested more than once every loop-start and end. This is done to circumvent debugging of the loop-control structures ;). Its just for approximate progress information of the operator anyway.
        nSteps = round(nSteps,0)
        curStep = 0
        lastProgressDisplayTime = 0
        #just for displaying the status--------------
#----------------------------- END INIT VALUES END ------------------------------------------
        depthLowerBoundReached = False
        depthUpperBoundReached = False
        #-------------------------------DEPTH_LOOP-----------------------------
        while( not (depthLowerBoundReached and depthUpperBoundReached)):
            heightLowerBoundReached = False
            heightUpperBoundReached = False
        #-------------------------------HEIGHT_LOOP-----------------------------
            while( not (heightLowerBoundReached and heightUpperBoundReached)):
                elevationLowerBoundReached = False
                elevationUpperBoundReached = False
        #-------------------------------ELEVATION_LOOP-----------------------------
                while( not (elevationLowerBoundReached and elevationUpperBoundReached)):
                    elevationAngleRad = numpy.deg2rad(elevationAngle + elevationAngleOffset)
                    rollLowerBoundReached = False
                    rollUpperBoundReached = False
        #-------------------------------ROLL_LOOP-----------------------------
                    while(not (rollLowerBoundReached and rollUpperBoundReached)):
                        rollAngleRad = numpy.deg2rad(rollAngle + rollAngleOffset)
                        azimuthLowerBoundReached = False
                        azimuthUpperBoundReached = False
        #-------------------------------AZIMUT_LOOP-----------------------------
                        while(not (azimuthLowerBoundReached and azimuthUpperBoundReached)):
                            azimuthAngleRad = numpy.deg2rad(azimuthAngle + azimuthAngleOffset)
                            #---------------------- ROTATING IN DESIRED POSE AND TEST IK SOLUTION ---------------------
                            try:

                               azimuthAngleOffsetRad = numpy.deg2rad(azimuthAngleOffset)
                               rollAngleOffsetRad = numpy.deg2rad(rollAngleOffset)
                               elevationAngleOffsetRad = numpy.deg2rad(elevationAngleOffset)

                               nextIterationOffsetTransform = generateHomogenTransformationMatrix(depthOffset,0.0,heightOffset,rollAngleOffsetRad, elevationAngleOffsetRad, azimuthAngleOffsetRad)
                               nextIterationTargetTransform = targetTransform * nextIterationOffsetTransform
                               
                               #TODO this offset has to be an parameter
                               #cameraOffset = 0.161
                               #cameraOffset = 0.0
                               #TODO: for UR6 take the offsets into account: #targetTransform = generateHomogenTransformationMatrix(targetLoc3D[0],targetLoc3D[1],targetLoc3D[2], numpy.deg2rad(azimuth-90), numpy.deg2rad(elevation-90), numpy.deg2rad(roll))
                               #robotToManipTransform = generateHomogenTransformationMatrix(cameraOffset,0.0,0.0,pi/2, 0,pi/2)
                               #nextFinalIterTransformManip = nextIterationTargetTransform * robotToManipTransform 
                               
                               
                               
                               nextFinalIterTransformManip_array = numpy.array(nextIterationTargetTransform[:][:])
                               
                            except Exception as ex:
                               print >> sys.stderr, "-----------------------------------------------------"
                               print >> sys.stderr, "ERROR calculating the goal parameters for the next Iteration solution"
                               print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
                               print >> sys.stderr, "additional informations (if any): " +  str(ex)
                               print >> sys.stderr, "-----------------------------------------------------"
                                   
                            solution = self.searchIKSolution(nextFinalIterTransformManip_array)
                            curStep += 1
                            if (type(solution) !=  type(1)):
                                self.setJoints(solution);
                                pose6D = self.robot.GetActiveManipulator().GetEndEffectorTransform().tolist()
                                pose6D = self.convertFromManipIKtoZAFHCoords(pose6D).tolist()
                                self.setJoints(currentJoints);
                                stdout.write("\n")
                                print "Found a position by iteration that allows at least one IK solution. Position :" + str(planningTarget)
                                print "changed depth by :" + str(depthOffset) + " m"
                                print "changed height by :" + str(heightOffset) + " m"
                                print "resulting elevation Angle :" + str(elevationAngle) + " degree"
                                print "resulting roll Angle :" + str(rollAngle) + " degree"
                                print "resulting azimuth Angle :" + str(azimuthAngle) + " degree"
                                print "resulting 6D pose is :" + str(pose6D)
                                print "IK solution: " + str(solution)
                                return solution, pose6D
                            if verbose:
                                print "--------------------------------------------------------------------"
                                print "Unable to find an IK solution at Position :" + str(planningTarget)
                                print ("%.2f%% possible combinations were tested (%.0f of %.0f)" %((curStep/nSteps)*100, curStep, nSteps))
                                print "changed depth by :" + str(depthOffset) + " m"
                                print "changed height by :" + str(heightOffset) + " m"
                                print "changed azimuth Angle by :" + str(azimuthAngleOffset) + " degree"
                                print "changed elevation Angle by :" + str(elevationAngleOffset) + " degree"
                                print "changed roll Angle by :" + str(rollAngleOffset) + " degree"
                                print "elevation Angle :" + str(elevationAngle) + " degree"
                                print "roll Angle :" + str(rollAngle) + " degree"
                                print "azimuth Angle :" + str(azimuthAngle) + " degree"
                                print "--------------------------------------------------------------------"
                                tcpAxisBody.SetTransform(nextFinalIterTransformManip_array)
                                self.env.AddKinBody(tcpAxisBody)
                                time.sleep(0.2)
                                self.env.Remove(tcpAxisBody)
                            else:
                                #just for displaying the status--------------
                                if (lastProgressDisplayTime+1 < time.time() - startTime):
                                    stdout.write("\r%.2f%% possible combinations were tested (%.0f of %.0f)" %((curStep/nSteps)*100, curStep, nSteps))
                                    stdout.flush()
                                    lastProgressDisplayTime = time.time() - startTime
                                #just for displaying the status--------------
                                #determine loop conditions for azimuth
                            if( abs(azimuthUpModificator) >=  abs(azimuthUpperBound)):
                                azimuthUpperBoundReached = True
                            if( abs(azimuthDownModificator) >=  abs(azimuthLowerBound)):
                                azimuthLowerBoundReached = True

                            if(azimuthUpperBoundReached and azimuthLowerBoundReached):
                                azimuthUpModificator = 0
                                azimuthDownModificator = 0
                                azimuthSwitch = 0
                                azimuthAngle = azimuth
                                azimuthAngleOffset = 0
                            else:
                                if(0 == (azimuthSwitch % 2)):#uppper cycle
                                    #print "upper cycle"
                                    if( not azimuthUpperBoundReached):
                                        azimuthUpModificator += azimuthAngleStepSize
                                        azimuthAngleOffset = azimuthUpModificator
                                    if( azimuthUpperBoundReached and (not azimuthLowerBoundReached)): #if we already reached the upperBound, but not yet the lower, just perform a lower cycle instead
                                        azimuthDownModificator -= azimuthAngleStepSize
                                        azimuthAngleOffset = azimuthDownModificator

                                if(1 == (azimuthSwitch % 2)):#lower cycle,
                                    #print "lower cycle"
                                    if( not azimuthLowerBoundReached):
                                        azimuthDownModificator -= azimuthAngleStepSize
                                        azimuthAngleOffset = azimuthDownModificator
                                    if( azimuthLowerBoundReached and (not azimuthUpperBoundReached)): #if we already reached the lowerBound, but not yet the upper, just perform a upper cycle instead
                                        azimuthUpModificator += azimuthAngleStepSize
                                        azimuthAngleOffset = azimuthUpModificator
                                azimuthSwitch += 1
            #--------------------------END--AZIMUT_LOOP--END------------------------
                        if( abs(rollUpModificator) >=  abs(rollUpperBound)):
                            rollUpperBoundReached = True
                        if( abs(rollDownModificator) >=  abs(rollLowerBound)):
                            rollLowerBoundReached = True

                        if(rollUpperBoundReached and rollLowerBoundReached):
                            rollUpModificator = 0
                            rollDownModificator = 0
                            rollAngle = roll
                            rollSwitch = 0
                            rollAngleOffset = 0
                        else:
                            if(0 == (rollSwitch % 2)):#uppper cycle
                                #print "upper cycle"
                                if( not rollUpperBoundReached):
                                    rollUpModificator += rollAngleStepSize
                                    rollAngleOffset = rollUpModificator
                                if( rollUpperBoundReached and (not rollLowerBoundReached)): #if we already reached the upperBound, but not yet the lower, just perform a lower cycle instead
                                    rollDownModificator -= rollAngleStepSize
                                    rollAngleOffset = rollDownModificator

                            if(1 == (rollSwitch % 2)):#lower cycle,
                                #print "lower cycle"
                                if( not rollLowerBoundReached):
                                    rollDownModificator -= rollAngleStepSize
                                    rollAngleOffset = rollDownModificator
                                if( rollLowerBoundReached and (not rollUpperBoundReached)): #if we already reached the lowerBound, but not yet the upper, just perform a upper cycle instead
                                    rollUpModificator += rollAngleStepSize
                                    rollAngleOffset = rollUpModificator
                            rollSwitch += 1
        #--------------------------END--ROLLOOP--END------------------------
                    if( abs(elevationUpModificator) >=  abs(elevationUpperBound)):
                        elevationUpperBoundReached = True
                    if( abs(elevationDownModificator) >=  abs(elevationLowerBound)):
                        elevationLowerBoundReached = True

                    if(elevationUpperBoundReached and elevationLowerBoundReached):
                        elevationUpModificator = 0
                        elevationDownModificator = 0
                        elevationAngle = elevation
                        elevationSwitch = 0
                        elevationAngleOffset = 0
                    else:
                        if(0 == (elevationSwitch % 2)):#uppper cycle
                            #print "upper cycle"
                            if( not elevationUpperBoundReached):
                                elevationUpModificator += elevationAngleStepSize
                                elevationAngleOffset = elevationUpModificator
                            if( elevationUpperBoundReached and (not elevationLowerBoundReached)): #if we already reached the upperBound, but not yet the lower, just perform a lower cycle instead
                                elevationDownModificator -= elevationAngleStepSize
                                elevationAngleOffset = elevationDownModificator

                        if(1 == (elevationSwitch % 2)):#lower cycle,
                            #print "lower cycle"
                            if( not elevationLowerBoundReached):
                                elevationDownModificator -= elevationAngleStepSize
                                elevationAngleOffset = elevationDownModificator
                            if( elevationLowerBoundReached and (not elevationUpperBoundReached)): #if we already reached the lowerBound, but not yet the upper, just perform a upper cycle instead
                                elevationUpModificator += elevationAngleStepSize
                                elevationAngleOffset = elevationUpModificator
                        elevationSwitch += 1
        #--------------------------END--ELEVATIONLOOP--END------------------------
                if( abs(heightUpModificator) >=  abs(heightUpperBound)):
                    heightUpperBoundReached = True
                if( abs(heightDownModificator) >=  abs(heightLowerBound)):
                    heightLowerBoundReached = True

                if(heightUpperBoundReached and heightLowerBoundReached):
                    heightUpModificator = 0
                    heightDownModificator = 0
                    planningTarget[2] = targetLoc3D[2]
                    heightOffset = 0
                    heightSwitch = 0
                else:
                    if(0 == (heightSwitch % 2)):#uppper cycle
                        #print "upper cycle"
                        if( not heightUpperBoundReached):
                            heightUpModificator += heightStepSize
                            heightOffset = heightUpModificator
                            planningTarget[2] = targetLoc3D[2] + heightUpModificator
                        if( heightUpperBoundReached and (not heightLowerBoundReached)): #if we already reached the upperBound, but not yet the lower, just perform a lower cycle instead
                            heightDownModificator -= heightStepSize
                            heightOffset = heightDownModificator
                            planningTarget[2] = targetLoc3D[2] + heightDownModificator

                    if(1 == (heightSwitch % 2)):#lower cycle,
                        #print "lower cycle"
                        if( not heightLowerBoundReached):
                            heightDownModificator -= heightStepSize
                            heightOffset = heightDownModificator
                            planningTarget[2] = targetLoc3D[2] + heightDownModificator
                        if( heightLowerBoundReached and (not heightUpperBoundReached)): #if we already reached the lowerBound, but not yet the upper, just perform a upper cycle instead
                            heightUpModificator += heightStepSize
                            heightOffset = heightUpModificator
                            planningTarget[2] = targetLoc3D[2] + heightUpModificator
                    heightSwitch += 1
        #--------------------------END--HEIGHTLOOP--END------------------------
            if( abs(depthUpModificator) >=  abs(depthUpperBound)):
                depthUpperBoundReached = True
            if( abs(depthDownModificator) >=  abs(depthLowerBound)):
                depthLowerBoundReached = True

            if(depthUpperBoundReached and depthLowerBoundReached):
                depthUpModificator = 0
                depthDownModificator = 0
                depthOffset = 0
                depthSwitch = 0
            else:
                if(0 == (depthSwitch % 2)):#uppper cycle
                    #print "upper cycle"
                    if( not depthUpperBoundReached):
                        depthUpModificator += depthStepSize
                        depthOffset = depthUpModificator
                    if( depthUpperBoundReached and (not depthLowerBoundReached)): #if we already reached the upperBound, but not yet the lower, just perform a lower cycle instead
                        depthDownModificator -= depthStepSize
                        depthOffset = depthDownModificator

                if(1 == (depthSwitch % 2)):#lower cycle,
                    #print "lower cycle"
                    if( not depthLowerBoundReached):
                        depthDownModificator -= depthStepSize
                        depthOffset = depthDownModificator
                    if( depthLowerBoundReached and (not depthUpperBoundReached)): #if we already reached the lowerBound, but not yet the upper, just perform a upper cycle instead
                        depthUpModificator += depthStepSize
                        depthOffset = depthUpModificator
                depthSwitch += 1
        #--------------------------END--DEPTHLOOP--END------------------------
        


        duration = time.time() - startTime
        stdout.write("\n")
        print "Iteration exceeded all possibilities, no solution could be found"
        print "duration: " + str(round(duration,2)) + " s"
        print "additional informations (if any):"
        print "target position was: " + str(planningTarget)
        print "-----------------------------------------------------"
        self.setJoints(currentJoints)
        return None, None #!!!IMPORTANT!!! always return the same amount of return values if the function is callable from the c++ interface -> else segfault

    def getEndEffectorTransform(self):
        return self.robot.GetActiveManipulator().GetEndEffectorTransform()

    def getCurrentORJoints(self):
        return self.robot.GetActiveDOFValues()


## iterates for a IK solution at the specified position with the specified iteration parameters
## this iteration variant iterates from the mid of the point and the angle values
## explanation for iteration behavior based of corresponding upper and lower bound values:
##         xxxFrom = -0.05, xxxTo = 0.05 ==> will iterate in the specified step sizes from the mid to the outer boundaries (0, 0.5, -0.5 ...)
##         xxxFrom = -0.05, xxxTo = 0.08 ==> will iterate as the above starting from 0. After one boundary is reached, it will be iterated only in the other boundaries' direction
##      xxxFrom =  0.05, xxxTo = 0.08 ==> if both boundaries have the same algebraic sign, it will be symmetrical iterated from the middle to both boundaries.
##        Please note the possible loss of steps due to fractions from the determination of the middle value (which will be rounded to 3 decimals = 1 mm)
##        xxxFrom =  0.08, xxxTo = 0.05 ==> does the same as the above. It is not necessary to specify the xxxFrom boundary with a lower value than the xxxTo boundary
## iteration hierarchy:
##                         1. azimuth
##                         2. roll
##                         3. elevation
##                        4. height
##                        5. depth
##    Meaning for example all azimuth angles are tried before the next roll angle is tried
#    def iterForIKSolutions(self, params):
#        try:
#            targetLoc3D             = params['targetLoc3D']
#            elevation               = params['elevation']
#            roll                    = params['roll']
#            azimuth                 = params['azimuth']
#
#            depthFrom              = params['depthFrom']
#            depthTo              = params['depthTo']
#            heightFrom             = params['heightFrom']
#            heightTo             = params['heightTo']
#            elevationAngleFrom     = params['elevationAngleFrom']
#            elevationAngleTo     = params['elevationAngleTo']
#            rollAngleFrom          = params['rollAngleFrom']
#            rollAngleTo          = params['rollAngleTo']
#            azimuthAngleFrom        = params['azimuthAngleFrom']
#            azimuthAngleTo        = params['azimuthAngleTo']
#
#            depthStepSize           = params['depthStepSize']               # meter
#            heightStepSize          = params['heightStepSize']              # meter
#            elevationAngleStepSize  = params['elevationAngleStepSize']      # +- degree
#            rollAngleStepSize       = params['rollAngleStepSize']           # +- degree
#            azimuthAngleStepSize     = params['azimuthAngleStepSize']         # +- degree
#            verbose                 = params['verbose']
#
#        #--------------------- catch possible errors ------------------------------
#        except KeyError as ex:
#            print >> sys.stderr, "-----------------------------------------------------"
#            print >> sys.stderr, "ERROR: missing parameter in the parameter list :" + str(ex)
#            print >> sys.stderr, "additional informations (if any): None"
#            print >> sys.stderr, "-----------------------------------------------------"
#            return None, None
#       
#        try:
#            tcpAxisBody = self.env.ReadKinBodyURI("models/axes/coordAxis.kinbody.xml")
#            tcpAxisBody.SetName("coordCross")
#        except Exception as ex:
#            print >> sys.stderr, "-----------------------------------------------------"
#            print >> sys.stderr, "ERROR importing coordCross"
#            print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
#            print >> sys.stderr, "additional informations (if any): " +  str(ex)
#            print >> sys.stderr, "-----------------------------------------------------"
#            return None, None
#
#        print "target location is:" + str(targetLoc3D)
#        try:
#            for key in params:
#                if params[key] is None:
#                    if not ((key == 'maxtries') or (key == 'maxiter')):
#                        print >> sys.stderr, "Parameter: " + str(key) +" must not be unspecified"
#                        return None, None
#        except Exception as ex:
#                print >> sys.stderr, "-----------------------------------------------------"
#                print >> sys.stderr, "ERROR :" + str(sys.exc_info()[0])
#                print >> sys.stderr, "additional informations (if any): " +  str(ex)
#                print >> sys.stderr, "-----------------------------------------------------"
#                return None, None
#        #--------------------- END catch possible errors ------------------------------
#
#        #--------------------- init values  ------------------------------
#
#        depthOffset = depthFrom
#        heightOffset = heightFrom
#        planningTarget = list(targetLoc3D)          #make a copy of the original target used for iterating and planning
#        elevationAngleOffset  = elevationAngleFrom # in x-z plane
#        rollAngleOffset  = rollAngleFrom           # in y-z plane
#        azimuthAngleOffset = azimuthAngleFrom        # in x-y plane
#
#
#        nextDepth = True
#        nextHeight = True
#        nextElevationAngle  = True
#        nextRollAngle  = True
#        nextAzimuthAngle  = True
#
#        currentJoints = self.robot.GetActiveDOFValues()
#        #--------------------- END init values  ------------------------------
#
#        #-------------------------------DEPTHLOOP-----------------------------
#        while( nextDepth ):
#            azimuthAngleRad = numpy.deg2rad(azimuth + azimuthAngleOffset)
#            nextHeight = True
#        #-------------------------------HEIGHTLOOP-----------------------------
#            while( nextHeight):
#                planningTarget[2] = targetLoc3D[2] + heightOffset
#                nextElevationAngle = True
#        #-------------------------------ELEVATIONLOOP-----------------------------
#                while(nextElevationAngle):
#                    elevationAngleRad = numpy.deg2rad(elevation + elevationAngleOffset)
#                    elevationMatrix = numpy.matrix([[1, 0, 0, 0],
#                                            [0, numpy.cos(elevationAngleRad), -numpy.sin(elevationAngleRad), 0],
#                                            [0, numpy.sin(elevationAngleRad), numpy.cos(elevationAngleRad), 0],
#                                            [0, 0, 0, 1]
#                                            ])
#                    nextRollAngle = True
#        #------------------------------- ROLLLOOP-----------------------------
#                    while(nextRollAngle):
#                        rollAngleRad = numpy.deg2rad(roll + rollAngleOffset)
#                        rollAngleMatrix = numpy.matrix([[numpy.cos(rollAngleRad),-numpy.sin(rollAngleRad), 0,0],
#                                                    [numpy.sin(rollAngleRad), numpy.cos(rollAngleRad), 0,0],
#                                                    [0, 0, 1, 0],
#                                                    [0, 0, 0, 1]
#                                                    ])
#                        nextAzimuthAngle = True
#     #------------------------------- Azimuth LOOP-----------------------------
#                        while(nextAzimuthAngle):
#                            azimuthAngleRad = numpy.deg2rad(azimuth + azimuthAngleOffset)
#                            azimuthMatrix = numpy.matrix([[numpy.cos(azimuthAngleRad),-numpy.sin(azimuthAngleRad), 0, 0],
#                                            [numpy.sin(azimuthAngleRad), numpy.cos(azimuthAngleRad), 0, 0],
#                                            [0, 0, 1, 0],
#                                            [0, 0, 0, 1]
#                                            ])
#                            #---------------------- ROTATING IN DESIRED POSE AND TEST IK SOLUTION ---------------------
#                            transform = (azimuthMatrix * elevationMatrix * rollAngleMatrix)
#                            zDirVec = numpy.matrix([ [0], [0], [1], [1] ])
#                            zDirVec = transform * zDirVec
#                            zDirVec[2] = 0
#                            zDirVec /= linalg.norm(zDirVec[0][0:3])
#                            zDirVec = zDirVec * depthOffset
#                            zDirVec = numpy.array(zDirVec[:][:])
#                            transform = numpy.array(transform[:][:])
#                            transform[0][3] = planningTarget[0] + zDirVec[0]
#                            transform[1][3] = planningTarget[1] + zDirVec[1]
#                            transform[2][3] = planningTarget[2] + zDirVec[2]
#                            solution = self.searchIKSolution(transform)
#                            if (type(solution) !=  type(1)):
#                                self.setJoints(solution);
#                                pose6D = self.robot.GetActiveManipulator().GetEndEffectorTransform().tolist()
#                                pose6D = self.convertFromManipIKtoZAFHCoords(pose6D).tolist()
#                                self.setJoints(currentJoints);
#                                print "Found a position by iteration that allows at least one IK solution. Position :" + str(planningTarget)
#                                print "changed depth by :" + str(depthOffset) + " m"
#                                print "changed height by :" + str(heightOffset) + " m"
#                                print "resulting elevation Angle :" + str(elevation + elevationAngleOffset) + " degree"
#                                print "resulting roll Angle :" + str(roll + rollAngleOffset) + " degree"
#                                print "resulting azimuth Angle :" + str(azimuth + azimuthAngleOffset) + " degree"
#                                print "resulting 6D pose is :" + str(pose6D)
#                                print "solution: " + str(solution)
#                                return solution, pose6D
#                            if verbose:
#                                print "--------------------------------------------------------------------"
#                                print "Unable to find an IK solution at Position :" + str(planningTarget)
#                                print "changed depth by :" + str(depthOffset) + " m"
#                                print "changed height by :" + str(heightOffset) + " m"
#                                print "elevation Angle :" + str(elevation + elevationAngleOffset) + " degree"
#                                print "roll Angle :" + str(roll + rollAngleOffset) + " degree"
#                                print "azimuth Angle :" + str(azimuth + azimuthAngleOffset) + " degree"
#                                print "--------------------------------------------------------------------"
#                                tcpAxisBody.SetTransform(transform)
#                                self.env.AddKinBody(tcpAxisBody)
#                                #self.moveKinbody("coordCross", transform)
#                                time.sleep(0.2)
#                                self.env.Remove(tcpAxisBody)
#                            #---------------------- END ROTATING IN DESIRED POSE AND TEST IK SOLUTION ---------------------
#                            if azimuthAngleFrom < azimuthAngleTo:
#                                azimuthAngleOffset += azimuthAngleStepSize
#                                nextAzimuthAngle = (azimuthAngleOffset <= azimuthAngleTo)
#                            else:
#                                azimuthAngleOffset -= azimuthAngleStepSize
#                                nextAzimuthAngle = (azimuthAngleOffset >= azimuthAngleTo)
#                            if not nextAzimuthAngle:
#                                azimuthAngleOffset = azimuthAngleFrom
#    #---------------------------------END AZIMUT ANGLE ITERATION---------------------------------------------------------------
#                        if rollAngleFrom < rollAngleTo:
#                            rollAngleOffset += rollAngleStepSize
#                            nextRollAngle = (rollAngleOffset <= rollAngleTo)
#                        else:
#                            rollAngleOffset -= rollAngleStepSize
#                            nextRollAngle = (rollAngleOffset >= rollAngleTo)
#                        if not nextRollAngle:
#                            rollAngleOffset = rollAngleFrom
##---------------------------------END ROLL ANGLE ITERATION---------------------------------------------------------------
#                    if elevationAngleFrom < elevationAngleTo:
#                        elevationAngleOffset += elevationAngleStepSize
#                        nextElevationAngle = (elevationAngleOffset <= elevationAngleTo)
#                    else:
#                        elevationAngleOffset -= elevationAngleStepSize
#                        nextElevationAngle = (elevationAngleOffset >= elevationAngleTo)
#                    if not nextElevationAngle:
#                        elevationAngleOffset = elevationAngleFrom
#        #--------------------------END--ELEVATIONLOOP--END------------------------
#                if heightFrom < heightTo:
#                    heightOffset += heightStepSize
#                    nextHeight = (heightOffset <= heightTo)
#                else:
#                    heightOffset -= heightStepSize
#                    nextHeight = (heightOffset >= heightTo)
#                if not nextHeight:
#                    heightOffset = heightFrom
#        #--------------------------END--HEIGHTLOOP--END------------------------
#            if depthFrom < depthTo:
#                depthOffset += depthStepSize
#                nextDepth = (depthOffset <= depthTo)
#            else:
#                depthOffset -= depthStepSize
#                nextDepth = (depthOffset >= depthTo)
#            if not nextDepth:
#                depthOffset = depthFrom
#        #--------------------------END--DEPTHLOOP--END------------------------
#        print >> sys.stderr, "-----------------------------------------------------"
#        print >> sys.stderr, "UNABLE to find a possible IK solution by iteration"
#        print >> sys.stderr, "additional informations (if any):"
#        print >> sys.stderr, "target position was: " + str(planningTarget)
#        print >> sys.stderr, "-----------------------------------------------------"
#        self.setJoints(currentJoints)
#        return None, None #!!!IMPORTANT!!! always return the same amount of return values if the function is callable from the c++ interface -> else segfault


##class that provides easy usage of the openRave graspPlanning functionality
#class GraspManipulation:
#    ikmodel = None
#    gmodel = None
#    taskmanip = None
#    robot = None
#
#    def __init__(self, robot):
#        self.robot=robot
#        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.TranslationDirection5D)
#        self.basemanip = interfaces.BaseManipulation(self.robot)
#        #self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
#        if not self.ikmodel.load():
#            self.ikmodel.autogenerate()
#
#
#    def moveToolCenterByGraspPlanningTo(self, target, dests=None):
#        self.gmodel = databases.grasping.GraspingModel(robot=self.robot,target=target)
#        if not self.gmodel.load():
#            self.gmodel.autogenerate()
#        self.taskmanip = interfaces.TaskManipulation(self.robot,graspername=self.gmodel.grasper.plannername)
#
#        self.robot.SetActiveManipulator(self.gmodel.manip)
#        self.robot.SetActiveDOFs(self.gmodel.manip.GetArmIndices())
#
#        try:
#            goals,graspindex,searchtime,trajdata = self.taskmanip.GraspPlanning(graspindices=self.gmodel.graspindices,grasps=self.gmodel.grasps[:],
#                                                                                            target=target,approachoffset=0.2,destposes=dests,
#                                                                                            seedgrasps = 20,seeddests=1,seedik=20,maxiter=1500,
#                                                                                            randomgrasps=False,randomdests=True)
#            self.robot.WaitForController(0)
#        except Exception as ex:
#            print "\nGraspplanning Error: ", ex
#            return None
#        return [goals,graspindex,searchtime,trajdata]
#
#    def graspObject(self, target, dests=None):
#        planningResult = self.moveToolCenterByGraspPlanningTo(target, dests=dests)
#        if(planningResult is not None):
#            try:
#                self.taskmanip.CloseFingers()
#                self.robot.WaitForController(0)
#                self.robot.Grab(target)
#            except Exception as ex:
#                print "\nError during grasping the target: ", target, ex
#        return planningResult
#
#    def graspAndPlaceObject(self, target, dests):
#        planningResult = self.graspObject(target, dests=dests)
#        if(planningResult is not None):
#            try:
#                self.basemanip.MoveToHandPosition(ikparams=planningResult[0],maxiter=1000,maxtries=1,seedik=4)
#                self.robot.WaitForController(0)
#            except planning_error,e:
#                print 'failed to reach a goal',e
#        else:
#            print "unable to place object, previous planning failed"
#
#
#
#    def releaseObject(self, target):
#        self.taskmanip.ReleaseFingers(target=target)
#        self.robot.ReleaseAllGrabbed()
#        self.robot.WaitForController(0)

