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

def serializeList(list):
    resultString = ""
    for f in list:
        resultString = resultString + ('%.5f '%(f))
    return resultString

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

class CoMPSSimpleManipulation:
    orUtil = None
    
    def __init__(self, orUtil):
        self.orUtil = orUtil

    def displayCoodCross(self, transform, name):
        body = self.orUtil.env.ReadKinBodyXMLFile('models/axes/coordAxis.kinbody.xml')
        body.SetName(name)
        self.orUtil.env.AddKinBody(body)
        body.SetTransform(transform.tolist())
        return body
    
    def getOffsetWorld_TCP(self):
        #get the TSR's offset frame in w coordinates (offset of TSR-Frame T0_w to the endeffector's TCP of the manipulator)
        tmp = rodrigues([0, pi/2, 0])*rodrigues([0, 0, pi/2]) #constrain the mounting plate facing upwards
        return MakeTransform(tmp,mat([0,0,0]).T) #offset is just a rotation as the TCP is between the fingers
    
    def sampleIKForPose(self, params):
        try:
            self.orUtil.env.GetRobot(self.orUtil.robot.GetName()).SetActiveDOFs(self.orUtil.maskWithoutGripper)
            currentJoints = self.orUtil.getCurrentORJoints()
            probs_cbirrt = RaveCreateProblem(self.orUtil.env,'CBiRRT')
            self.orUtil.env.LoadProblem(probs_cbirrt,self.orUtil.robot.GetName())
            #pi/2 auf z
            #rotTarget = rodrigues([numpy.deg2rad(params['roll']), numpy.deg2rad(params['elevation']), numpy.deg2rad(params['azimuth'])])
            targetTransform = MakeTransform(rodrigues([0,0,0]),mat(params['targetLoc3D']).T)
            distToTarget = (params['targetLoc3D'][0]*params['targetLoc3D'][0]) + (params['targetLoc3D'][1]*params['targetLoc3D'][1]) + (params['targetLoc3D'][2]*params['targetLoc3D'][2])
            print("Distance to Target: ", distToTarget)
            #targetTransform = MakeTransform(rotTarget,mat(params['targetLoc3D']).T)
            #definde the boundaries. Order --> depth, side, height, roll, elevation, azimuth
            boundaries1 = mat([ 
                                0,                                              0,
                                0,                                              0,
                                params['heightFrom'],                           params['heightTo'],   
                                numpy.deg2rad(params['rollAngleFrom']),         numpy.deg2rad(params['rollAngleTo']),
                                numpy.deg2rad(params['elevationAngleFrom']),    numpy.deg2rad(params['elevationAngleTo']),   
                                numpy.deg2rad(params['azimuthAngleFrom']),      numpy.deg2rad(params['azimuthAngleTo'])])
            
            boundaries2 = mat([
                                params['depthFrom'],            params['depthTo'],
                                params['sideFrom'],             params['sideTo'],
                                0,                              0,   
                                0,                              0,
                                0,                              0,   
                                0,                              0])

            TSRstring1 = SerializeTSR(0,'NULL',targetTransform,mat(eye(4)),boundaries1)
            TSRstring2 = SerializeTSR(0,'NULL',MakeTransform(rodrigues([0, 0, 0]),mat([0,0,0]).T),self.getOffsetWorld_TCP(),boundaries2)
            #Generate the TSRChain
            TSRChainString1 = SerializeTSRChain(0,1,0,2,TSRstring1 + " " + TSRstring2,'NULL',[])
            nTries = 3
            i = 0
            objectString = ""
            print "-----------------------------------------------------"
            print "Started Sampling For IK Solutions"
            print "-----------------------------------------------------"
            start = time.time()
            while i < nTries:
                self.orUtil.setJoints(currentJoints)
                if self.orUtil.objectInGripper is not None:
                    objectString = 'heldobject ' + self.orUtil.objectInGripper.GetName()
                retVal = probs_cbirrt.SendCommand('RunCBiRRT ' + objectString + ' filename cSampleIKtraj.txt psample 0.25 timelimit 3 %s'%TSRChainString1)
 #               targetTransform = MakeTransform(rodrigues([0,pi/2.0,0]),mat(params['targetLoc3D']).T)
 #               print("targetTransform: ", targetTransform)
 #               retVal = probs_cbirrt.SendCommand('DoGeneralIK exec nummanips 1 maniptm 0 %s'%SerializeTransform(targetTransform))
                #print("retVal: ", retVal)
                #retVal = retVal.partition(" ")
                if (int(retVal[0]) == 1):

                    with open("ikSolution.txt") as f:
                        content = f.readlines()
                        content = [x.strip() for x in content]
                    solution = [float(i) for i in content]
                    print("Solution: ", solution)
                    
                    #solution = trajRobot.Sample(trajRobot.GetDuration())[0:len(self.orUtil.maskWithoutGripper)]
                    
                    #print("trajRobot: ", trajRobot)
                    #print("Solution: ", solution)
                    self.orUtil.setJoints(solution)
                    pose6D = self.orUtil.robot.GetActiveManipulator().GetEndEffectorTransform().tolist()
                    pose6D = self.orUtil.convertFromManipIKtoZAFHCoords(pose6D).tolist()
                    self.orUtil.setJoints(currentJoints);
                    #duration = time.time() - start
                    #print "-----------------------------------------------------"
                    #print "Finished IK-Sampling"
                    #print "duration: " + str(round(duration,2)) + " s"
                    #print "-----------------------------------------------------"
                    return solution,pose6D
                else:
                     print "-----------------------------------------------------"
                     print "Failed to find IK-Solution in try " + str(i+1) + " of " +str(nTries)
                     print "Starting another try..."
                     print "-----------------------------------------------------"
                i += 1
            duration = time.time() - start
            self.orUtil.setJoints(currentJoints);
            print "Finished planning"
            print "duration: " + str(duration)
            print "-----------------------------------------------------"
            print "--------------------------------------------------------------------"
            print "Unable to find an IK solution at Position :" + str(targetTransform)
            print "Parameters were"
            print "Target Position: " + str(params['targetLoc3D'])
            print "Depth between:" + str(params['depthFrom']) +             " and " + str(params['depthTo']) +          " in meters"
            print "Height between:" + str(params['heightFrom']) +           " and " + str(params['heightTo']) +         " in meters"
            print "Roll between:" + str(params['rollAngleFrom']) +          " and " + str(params['rollAngleTo']) +      " in degree"
            print "Elevation between:" + str(params['elevationAngleFrom']) +" and " + str(params['elevationAngleTo']) + " in degree"
            print "Azimuth between:" + str(params['azimuthAngleFrom']) +    " and " + str(params['azimuthAngleTo']) +   " in degree"
            print "--------------------------------------------------------------------"
            return None,None
        except Exception as ex:
            self.orUtil.setJoints(currentJoints);
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR in sampleIKForPose of CoMPSSimpleManipulation: " + str(sys.exc_info()[0])
            print >> sys.stderr, "number of DOFs including gripper is: " + str(len(self.orUtil.maskWithGripper))
            print >> sys.stderr, "number of DOFs excluding gripper is: " + str(len(self.orUtil.maskWithoutGripper))
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------"
            return None,None
        
    #moves the joints using constrainedplanning to the specified angles
    #plans a path to the specified joint angles of the manipulator
    #parameter match the parameters needed for the 'BIRRT' planner
    #@param joints: list of the goal joint angles
    #@param params: parameter containing the boundaries of allowed movements of the TCP during the path
    #@param sampleTime: the timestep-length to sample the trajectory in seconds
    def moveJointsConstrained(self, joints, params, sampleTime):
        try:
            
            pathBoundaries1 = mat([  params['depthFrom'],            params['depthTo'],
                                    params['sideFrom'],             params['sideTo'],
                                    params['heightFrom'],           params['heightTo'],   
                                    0,                              0,
                                    0,                              0,   
                                    0,                              0])
            
            pathBoundaries2 = mat([ 0,                                              0,
                                    0,                                              0,
                                    0,                                              0,   
                                    numpy.deg2rad(params['rollAngleFrom']),         numpy.deg2rad(params['rollAngleTo']),
                                    numpy.deg2rad(params['elevationAngleFrom']),    numpy.deg2rad(params['elevationAngleTo']),   
                                    numpy.deg2rad(params['azimuthAngleFrom']),      numpy.deg2rad(params['azimuthAngleTo'])])

                                
            #note in order that the path boundaries are applied correctly we need the pose of the current TCP-pose in the world frame. This is why the rotation offset to the world is "subtracted" of the TCP transform
            TSRstringConstraint1 = SerializeTSR(0,'NULL',self.orUtil.getEndEffectorTransform()*linalg.inv(self.getOffsetWorld_TCP()),mat(eye(4)),pathBoundaries1)
            TSRstringConstraint2 = SerializeTSR(0,'NULL',mat(eye(4)),self.getOffsetWorld_TCP(),pathBoundaries2)
            TSRChainString = SerializeTSRChain(0,0,1,2,TSRstringConstraint1 + " " + TSRstringConstraint2,'NULL',[])
            
            
            probs_cbirrt = RaveCreateProblem(self.orUtil.env,'CBiRRT')
            self.orUtil.env.LoadProblem(probs_cbirrt,self.orUtil.robot.GetName())
            objectString = ""
            if self.orUtil.objectInGripper is not None:
                objectString = 'heldobject ' + self.orUtil.objectInGripper.GetName()
            if (int(probs_cbirrt.SendCommand('RunCBiRRT ' + objectString + ' jointgoals %d %s %s'%(size(joints), serializeList(joints),TSRChainString))) == 1):
                trajFile = open("cmovetraj.txt")
                trajRobot = RaveCreateTrajectory(self.orUtil.env, "")
                trajString = trajFile.read()
                trajFile.close()
                trajRobot.deserialize(trajString)
                return trajRobot
            else:
                print "--------------------------------------------------------------------"
                print "Unable to plan a constrained path to joint values :" + str(joints)
                print "Path boundaries were"
                print "Depth between:" + str(params['depthFrom']) +             " and " + str(params['depthTo']) +          " in meters"
                print "Side between:"  + str(params['sideFrom']) +               " and " + str(params['sideTo']) +          " in meters"
                print "Height between:" + str(params['heightFrom']) +           " and " + str(params['heightTo']) +         " in meters"
                print "Roll between:" + str(params['rollAngleFrom']) +          " and " + str(params['rollAngleTo']) +      " in degree"
                print "Elevation between:" + str(params['elevationAngleFrom']) +" and " + str(params['elevationAngleTo']) + " in degree"
                print "Azimuth between:" + str(params['azimuthAngleFrom']) +    " and " + str(params['azimuthAngleTo']) +   " in degree"
                print "--------------------------------------------------------------------"
                return None
                
        except Exception as ex:
            print >> sys.stderr, "-----------------------------------------------------"
            print >> sys.stderr, "ERROR in moveJointsConstrained: unable to plan constrained to specified joints: " + str(sys.exc_info()[0])
            print >> sys.stderr, "additional informations (if any): " +  str(ex)
            print >> sys.stderr, "-----------------------------------------------------" 
            return None
        return None
