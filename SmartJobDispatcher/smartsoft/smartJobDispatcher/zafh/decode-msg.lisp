;;--------------------------------------------------------------------------
;  BSD 3-Clause License
;
;  Copyright (C) Servicerobotics Ulm
;  University of Applied Sciences Ulm
;  Prittwitzstr. 10
;  89075 Ulm
;  Germany
;  All rights reserved.
;
;  Author: Matthias Lutz
;
;Redistribution and use in source and binary forms, with or without
;modification, are permitted provided that the following conditions are met:
;
;* Redistributions of source code must retain the above copyright notice, this
;  list of conditions and the following disclaimer.
;
;* Redistributions in binary form must reproduce the above copyright notice,
;  this list of conditions and the following disclaimer in the documentation
;  and/or other materials provided with the distribution.
;
;* Neither the name of the copyright holder nor the names of its
;  contributors may be used to endorse or promote products derived from
;  this software without specific prior written permission.
;
;THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
;DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
;FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
;DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
;SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
;CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
;OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;;--------------------------------------------------------------------------


(eval-when (:execute :load-toplevel :compile-toplevel)
	(require "cl-json")
	(load (format nil "split-sequence.lisp"))
)

(defun decode-msg (input-msg)
  "This function takes a string in json or s-expr format and decodes it to a alist"
  (let ((parsed-msg nil)
        (string-to-parse nil))
   (format t "Decode msg raw: ~s~%" input-msg)
   (if (wrapped-string-p input-msg)
     (setf string-to-parse (enroll-wrapped-string input-msg))
     (setf string-to-parse input-msg))
   (format t "Decode msg: ~s~%" string-to-parse)
   (format t "Typeof msg: ~a~%" (type-of string-to-parse))

    (handler-case
      (with-input-from-string (s string-to-parse)
        (let ((cl-json:*json-symbols-package* nil))
          (setf parsed-msg (cl-json:decode-json s))))
      (t (c)
        (format t "[decode-json-msg] WARNING decode msg json vaild try s-expr: ~a~%" c)
        (setf parsed-msg (decode-sexpr string-to-parse))
        (values nil c)))

    
    (format t "Parsed result ~s~%" parsed-msg)
    parsed-msg))





(defun decode-sexpr (input-msg)
  "This function takes a string in s-expr format and decodes it to a alist"
  (let ((request (read-from-string (concatenate 'string "(" input-msg ")")))
        (parsed-msg nil))
    (cond

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; POSITION HANDLING
      
      ;;;; Position and Station Handling
      ;; TeachPosition - OK
      ((equal (first request) 'TeachPosition)
        (let ((positions-list (split-sequence:SPLIT-SEQUENCE 'TeachPosition request :start 1 ))
              (result-list nil))
          (dolist (pose positions-list)
	    (multiple-value-bind (id x y yaw type)
                (apply #'values pose)
                (format t "Position ID: ~s  x: ~s  y: ~s phi: ~s type:~s ~%" id x y yaw type)
                (push `((POSITION-ID . ,id) (X . ,x) (Y . ,y) (PHI . ,yaw) (TYPE . ,(string-downcase (string type)))) result-list)))
          (push 'positions result-list)
          (setf parsed-msg `( (msg-type . "command") (command . ((type ."teach-position") ,result-list))))))


      ;; ReplacePositions - OK
      ((equal (first request) 'ReplacePositions)
       (cond   
          ((< (length request) 2)
            (setf parsed-msg `((msg-type . "command") (command . ((type ."replace-positions") (positions . ()))))))
          (T
            (let ((positions-list (split-sequence:SPLIT-SEQUENCE 'Position request :start 2 ))
                  (result-list nil))
              (dolist (pose positions-list)
                (multiple-value-bind (id x y yaw type)
                  (apply #'values pose)
                  (format t "Position ID: ~s  x: ~s  y: ~s phi: ~s type:~s ~%" id x y yaw type)
                  (push `((POSITION-ID . ,id) (X . ,x) (Y . ,y) (PHI . ,yaw) (TYPE . ,(string-downcase (string type)))) result-list)))
              (push 'positions result-list)
              (setf parsed-msg `( (msg-type . "command") (command . ((type ."replace-positions") ,result-list))))))))


      ;;TeachCurrentPosition - OK
      ((equal (first request) 'TeachCurrentPosition)
        (format t "TeachCurrentPosition robotinoid: ~s position: ~s ~%" (second request)(third request))
        (setf parsed-msg `( (msg-type . "command") (command . ((type . "teach-current-position") (robot-id . ,(second request)) (position-id . ,(third request)))))))


      ;;DeleteAllPositions - OK
      ((equal (first request) 'DeleteAllPositions)
        (format t "DeleteAllPositions~%")
        (setf parsed-msg `( (msg-type . "command") (command . ((type . "delete-all-positions"))))))


      ;; DeletePosition - OK
      ((equal (first request) 'DeletePosition)
        (format t "DeletePosition ID: ~s  ~%" (second request))
        (setf parsed-msg `( (msg-type . "command") (command . ((type . "delete-position") (position-id . ,(second request)))))))

      ;; POSITION HANDLING
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; STATION HANDLING

      ;; ReplaceStations - OK
      ((equal (first request) 'ReplaceStations)
        (cond   
          ((< (length request) 2)
            (setf parsed-msg `((msg-type . "command") (command . ((type ."replace-stations") (stations . ()))))))
          (T
            (let ((station-list (split-sequence:SPLIT-SEQUENCE 'Station request :start 2 ))
                  (result-list nil))
                (dolist (station station-list)
                  (multiple-value-bind (id x y yaw numBelts type docking-type approach-location)
                    (apply #'values station)
                    (format t "Station ID: ~s  x: ~s  y: ~s phi: ~s numBelts: ~s type: ~s docking-type: ~s app.Loc: ~s ~%" id x y yaw numBelts type docking-type approach-location)
                    (push `( (station-id . ,id) (X . ,x) (Y . ,y) (PHI . ,yaw) (num-belts . ,numBelts) 
                             (station-type . ,(string-downcase (string type))) (docking-type . ,(string-downcase (string docking-type))) (approach-location . ,approach-location) ) result-list)))
                (push 'stations result-list)
                (setf parsed-msg `((msg-type . "command") (command . ((type ."replace-stations") ,result-list))))))))

      ;;DeleteAllStations - OK
      ((equal (first request) 'DeleteAllStations)
        (format t "DeleteAllStations~%")
        (setf parsed-msg `((msg-type . "command") (command . ((type . "delete-all-stations"))))))


      ;; STATION HANDLING
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; MAPPING (SLAM)

      ;; StartMapping - OK
      ((equal (first request) 'StartMapping)
        (format t "StartMapping robotinoid: ~s mapname: ~s ~%" (second request)(third request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "start-mapping") (robot-id . ,(second request)) (mapname . ,(third request)))))))


      ;; AbortMapping - OK
      ((equal (first request) 'AbortMapping)
        (format t "AbortMapping robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "abort-mapping") (robot-id . ,(second request)))))))


      ;; StopMapping - OK
      ((equal (first request) 'StopMapping)
        (format t "StopMapping robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "stop-mapping") (robot-id . ,(second request)))))))

      ;; ReloadDefaultMap - OK
      ((equal (first request) 'ReloadDefaultMap)
        (format t "ReloadDefaultMap ~%")
        (setf parsed-msg `((msg-type . "command") (command . ((type . "reload-default-map"))))))


      ;; MAPPING (SLAM)
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; PATHS NAV HANDLING

      ;; TODO this is done via the robots, could be done directly accessing the PathNavigationServer
      ;; ClearAllPathNetworkNodes - OK
      ((equal (first request) 'ClearAllPathNetworkNodes)
        (format t "ClearAllPathNetworkNodes robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "clear-all-path-network-nodes") (robot-id . ,(second request)))))))

      ;; ReloadDefaultPaths - OK
      ((equal (first request) 'ReloadDefaultPaths)
        (format t "ReloadDefaultPaths THIS SOULD ONLY BE CALLED WHILE NONE!! OF THE ROBOTS IS NOT MOVEING~%")
        (setf parsed-msg `((msg-type . "command") (command . ((type . "reload-default-paths"))))))

      ;; PATHS NAV HANDLING
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; OTHER COMMANDS

      ;; SetRobotPose - OK
      ((equal (first request) 'SetRobotPose)
        (format t "SetRobotPose robotid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "set-robot-pose") (robot-id . ,(second request))  (x . ,(third request)) (y . ,(fourth request)) (phi . ,(fifth request)))))))

      ;; EndTask - OK
      ((equal (first request) 'EndTask)
        (format t "EndTask id: ~s~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "end-task") (robot-id . ,(second request)))))))


      ;; PauseRobot - OK
      ((equal (first request) 'PauseRobot)
	(format t "PauseRobot id: ~s~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "pause-robot") (robot-id . ,(second request)))))))

      ;; ContinueRobot - OK
      ((equal (first request) 'ContinueRobot)
	(format t "ContinueRobot id: ~s~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "continue-robot") (robot-id . ,(second request)))))))

      ;;; GotoPosition Deprecated - OK
      ((equal (first request) 'GotoPosition)
        (format t "GotoPosition robot-id: ~s pose-id: ~s~%" (second request) (third request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "goto-position") (robot-id . ,(second request)) (position-id . ,(third request)))))))

      ;;; PushCommand Deprecated - OK
      ((equal (first request) 'PushCommand)
	(format t "PushCommand id: ~s~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "push-command") (robot-id . ,(second request)) (pushed-command . ,(decode-sexpr-deprecated-commands (rest (cdr request)))))))))


      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; ROBOT STATE HANDLING


      ;; SetOperationMode - OK
      ((equal (first request) 'SetOperationMode)
        (format t "SetOperationMode robotid: ~s mode: ~s ~%" (second request)(third request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "set-operation-mode") (robot-id . ,(second request)) (mode . ,(string-downcase (string (third request)))))))))

      ;; AbortJobAndClearError - OK
      ((equal (first request) 'AbortJobAndClearError)
        (format t "AbortJobAndClearError robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "abort-job-and-clear-error") (robot-id . ,(second request)))))))

      ;; ClearError - OK
      ((equal (first request) 'ClearError)
        (format t "ClearError robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "clear-error") (robot-id . ,(second request)))))))

      ;; ManualAcknowledge - OK
      ((equal (first request) 'ManualAcknowledge)
        (format t "ManualAcknowledge robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "manual-acknowledge") (robot-id . ,(second request)))))))

      ;;ShutdownRobot - OK
      ((equal (first request) 'ShutdownRobot)
        (format t "ShutdownRobot robotinoid: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command") (command . ((type . "shutdown-robot") (robot-id . ,(second request)))))))

      ;;RestartSlaveRobots - OK
      ((equal (first request) 'RestartSlaveRobots)
        (format t "RestartSlaveRobots ~%")
        (setf parsed-msg `((msg-type . "command") (command . ((type . "restart-slave-robots"))))))


      ;;; FOR TESTING ONLY


      ;;;; DEBUG AND SPECIAL MSGS
      ((equal (first request) 'ON_COMPONENT_SHUTDOWN)
        (format t "ON_COMPONENT_SHUTDOWN~%")
        (setf parsed-msg `((msg-type . "debug-msg") (msg . ((type . "on-component-shutdown"))))))

      ;; Exit - OK
      ((equal (first request) 'Exit)
        (format t "Exit ~%")
        (setf parsed-msg `((msg-type . "debug-msg") (msg . ((type . "exit"))))))

      ;; Quit - OK
      ((equal (first request) 'Quit)
        (format t "Quit ~%")
        (setf parsed-msg `((msg-type . "debug-msg") (msg . ((type . "quit"))))))


      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; INFORMATION QUERY 

      ;; GetRobotIDMasterComponentsRunOn - OK
      ((equal (first request) 'GetRobotIDMasterComponentsRunOn)
        (format t "GetRobotIDMasterComponentsRunOn ip:~s~%" *MASTERIP*)
        (setf parsed-msg `((msg-type . "query") (query . ((type . "get-robot-id-master-master-components-run-on"))))))

      ;; GetAllRobotinoID --> handeled by FleetCom
      ;; GetAllPosition --> handeled by FleetCom
      ;; GetAllStation --> handeled by FleetCom
      ;; GetRobotInfo --> handeled by FleetCom 
      ;; GetJobInfo --> handeled by FleetCom
      ;; GetJobError --> handeled by FleetCom


      ;; INFORMATION QUERY
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; JOB HANDLING

      ;; PushJob
      ((equal (first request) 'PushJob)
        (format t "PushJob - type: ~s jobID: ~s priority: ~s robotid: ~s ~%" (second request)(third request)(fourth request) (fifth request))
          (cond
            ;; PushJob GotoPosition - OK
            ((equal (second request) 'GotoPosition) 
              (multiple-value-bind (jobtype jobid jobpriority robotid position-id)
                                   (values (second request) (third request) (fourth request) (fifth request) (sixth request))
              (format t "Goto Position: ~s ~%" position-id)
              (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "goto-position") (position-id . ,position-id)))))))

            ;; PushJob DeliverFromTo - OK
            ((equal (second request) 'DeliverFromTo)
              (multiple-value-bind (jobtype jobid jobpriority robotid from-station-id from-belt to-station-id to-belt)
                                   (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request))
              (format t "Deliver from Station: ~s Belt: ~s to Station ~s Belt ~s ~%" from-station-id from-belt to-station-id to-belt)
              (setf parsed-msg `(  (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "deliver-from-to") (from-station-id . ,from-station-id) (from-station-belt . ,from-belt) (to-station-id . ,to-station-id) (to-station-belt . ,to-belt)))))))

  

            ;; PushJob PalletizeItem JOBID PRIORITY ROBOTINOID STARTSTATION palletize-STATION DROP-OFF-STATION 
            ((equal (second request) 'PalletizeItem)
              (format t "PalletizeItem ~%")
              (multiple-value-bind (jobtype jobid jobpriority robotid start-station-id palletize-station-id drop-off-station-id)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request))
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "palletize-item") (start-station-id .  ,start-station-id) (palletize-station-id . ,palletize-station-id) (drop-off-station-id . ,drop-off-station-id)))))))
   

            ;; PushJob DeliverPallet JOBID PRIORITY ROBOTINOID palletize-STATION DROP-OFF-STATION
            ((equal (second request) 'DeliverPallet)
              (format t "DeliverFullPalet ~%")
              (multiple-value-bind (jobtype jobid jobpriority robotid palletize-station-id drop-off-station-id)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request))
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "deliver-pallet") (palletize-station-id . ,palletize-station-id) (drop-off-station-id . ,drop-off-station-id)))))))
                

            ;; PushJob RobotCommissioning JOBID PRIORITY ROBOTINOID COMMISSIONINGROBOT FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
            ((equal (second request) 'RobotCommissioning)
              (multiple-value-bind (jobtype jobid jobpriority robotid commissioningrobotid from-box-station-id from-box-belt to-station-id to-belt)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request) (tenth request))
              (format t "RobotCommissioning job: id ~s from Robot: ~s to Station ~s Belt ~s ~%" jobid commissioningrobotid to-station-id to-belt)
              (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) 
                (push . ((type . "commissioning") (commissioning-robot-id . ,commissioningrobotid) (from-box-station-id . ,from-box-station-id) (from-box-station-belt . ,from-box-belt) (to-station-id . ,to-station-id) (to-station-belt . ,to-belt) (order-items . (,((lambda () (setf tmp nil) (loop for (a b) on (nthcdr 10 request) by #'cddr do (push (list `(type . ,a)  `(quantity . ,b)) tmp)) tmp)) )  )))))))


            ;; PushJob FollowPerson JOBID PRIORITY ROBOTINOID
            ((equal (second request) 'FollowPerson)
              (multiple-value-bind (jobtype jobid jobpriority robotid)
                                    (values (second request) (third request) (fourth request) (fifth request))
                (format t "FollowPerson job ~%")
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "follow-person") ))))))


            ;; PushJob MPSDocking - OK
            ((equal (second request) 'MPSDocking)
              (multiple-value-bind (jobtype jobid jobpriority robotid action stationid tobelt)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request))
                (format t "MPSDocking job ~s action: ~s ~%" jobid action)
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "mps-docking") (action . ,(string-downcase (string action))) (station-id . ,stationid) (belt . ,tobelt)))))))


            ;; PushJob MPSLoading - OK
            ((equal (second request) 'MPSLoading)
              (multiple-value-bind (jobtype jobid jobpriority robotid action is-manual)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request))
                (format t "MPSLoading job ~s action: ~s ~%" jobid action)
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "mps-loading") (action . ,(string-downcase (string action))) (is-manual . ,is-manual)))))))


            ;; PushJob BatteryChargerDocking - OK
            ((equal (second request) 'BatteryChargerDocking)
              (multiple-value-bind (jobtype jobid jobpriority robotid action)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request))
                (format t "BatteryChargerDocking job ~s action: ~s ~%" jobid action)
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "battery-charger-docking") (action . ,(string-downcase (string action)))))))))
              

            ;; PushJob RobotGripper JOBID PRIORITY ROBOTINOID ACTION
            ((equal (second request) 'RobotGripper)
              (multiple-value-bind (jobtype jobid jobpriority robotid action)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request))
                (format t "RobotGripper job: ~s action: ~s ~%" jobid action)
                (setf parsed-msg `( (msg-type . "job") (action . "push-job") (job-id . ,jobid) (priority . ,jobpriority) (robot-id . ,robotid) (job . ((type . "robot-gripper") (action . ,(string-downcase (string action)))))))))
 
            (T
              (format t "Error parsing message JOB~%"))))

      ;; UpdateJob
      ((equal (first request) 'UpdateJob)
        (multiple-value-bind (jobtype jobid x y)
                             (values (second request) (third request) (fourth request) (fifth request))
          (format t "UpdateJob - type: ~s jobID: ~s ~%" jobtype jobid)
          (setf parsed-msg `(  (msg-type . "job") (action . "update-job") (job-id . ,jobid) (job . ((type . "follow-person") (action . ,"start-follow-person") (x . ,x) (y . ,y)))))))

      ;; EndJob - OK
      ((equal (first request) 'EndJob)
        (multiple-value-bind (jobid)
                             (values (second request))
          (format t "EndJob - jobID: ~s ~%" jobid)
          (setf parsed-msg `((msg-type . "job") (action . "end-job") (job-id . ,jobid)))))

      ;; DeleteJob - OK
      ((equal (first request) 'DeleteJob)
        (multiple-value-bind (jobid)
                             (values (second request))
          (format t "DeleteJob - jobID: ~s ~%" jobid)
          (setf parsed-msg `((msg-type . "job") (action . "delete-job") (job-id . ,jobid)))))



      (T
              (format t "Error parsing message~%")))
  parsed-msg))




(defun decode-sexpr-deprecated-commands (input-msg)
  "This function takes a string in s-expr format and decodes it to a alist, only handling the deprecated commands"
  (format t "decode-sexpr-deprecated-commands: ~s~%" input-msg)
  (let ((request input-msg)
        (parsed-msg nil))
    (cond

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; POSITION HANDLING
      
      ;;;; Position and Station Handling
      ;; DockTo - OK
      ((equal (first request) 'DockTo)
        (format t "DockTo: ~s ~%" (second request))
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "dock-to") (belt-id . ,(second request)))))))

      ;; Undock - OK
      ((equal (first request) 'Undock)
        (format t "Undock ~%")
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "undock"))))))

      ;; DockToCharger - OK
      ((equal (first request) 'DockToCharger)
        (format t "Undock ~%")
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "dock-to-charger"))))))

      ;; UnDockCharger - OK
      ((equal (first request) 'UnDockCharger)
        (format t "Undock ~%")
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "undock-from-charger"))))))

      ;; LoadBox - OK
      ((equal (first request) 'LoadBox)
        (format t "LoadBox ~%")
        (if (equal (second request) 1)
          (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "load-box") (load-manual . 1)))))
          (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "load-box") (load-manual . 0)))))))

      ;; UnloadBox - OK
      ((equal (first request) 'UnloadBox)
        (format t "UnloadBox ~%")
        (if (equal (second request) 1)
          (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "unload-box") (load-manual . 1)))))
          (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "unload-box") (load-manual . 0)))))))

      ;; GotoPosition - OK
      ((equal (first request) 'GotoPosition)
        (format t "GotoPosition request:~s ~%" request)
        (let ((robotid nil) (goal nil))
          (cond
            ((= (list-length request) 2)
              (setf robotid nil)
              (setf goal (second request)))
            ((= (list-length request) 3)
              (setf robotid (second request))
              (setf goal (third request)))
            (T
              (format t "Got illegal GotoPosition Message! ~s" request)))
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "goto-position") (robot-id . ,robotid) (position-id . ,goal)))))))

      ;; Exit - OK
      ((equal (first request) 'Exit)
        (format t "Exit ~%")
        (setf parsed-msg `((msg-type . "command-deprecated") (command . ((type . "Exit"))))))

      (T
              (format t "Error parsing message~%")))
  parsed-msg))


(defun keep-lisp-symbol-string (string)
  "Take a symbol name as a string and keep it lowercase"
  (string-downcase string))

(defun encode-msg (input-alist)
  "This function takes an alist and encodes it to a json object"
  (let ((json-msg nil))
    (format t "encode alist: ~a~%" input-alist)

    (handler-case
        (let ((cl-json:*lisp-identifier-name-to-json* #'keep-lisp-symbol-string)
              (str (make-string-output-stream)))
              (cl-json:encode-json input-alist str)
              (setf json-msg (get-output-stream-string str)))
      (t (c)
        (format t "[encode-msg] WARNING decode msg json vaild erro: ~a~%" c)
        (setf json-msg nil)
        (values nil c)))
    
    (format t "encode-msg encoded-msg: ~a~%" json-msg)
;; this is just doing (with-output-to-string!!
;;    (format t "encode-json-to-string msg ~a~%" (cl-json:encode-json-to-string json-msg))
    json-msg))


(defun enroll-wrapped-string (str)
 "This function removes a wrapped string"
 (read-from-string str))

;; Test cases
;(WRAPPED-STRING-P "TEST")
;(WRAPPED-STRING-P "\"Hallo\"")
;(WRAPPED-STRING-P "")

(defun wrapped-string-p (str)
  "This function checks if a string is wrapped by \"\""
  (if (and (stringp str) (> (length str) 0))
    (if (and (char= (aref str 0) #\") (char= (aref str (- (length str) 1)) #\"))
    T
    NIL)
    NIL))
