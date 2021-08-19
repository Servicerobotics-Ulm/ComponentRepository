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


;;-----------------------------------------------------------------------
;; THIS CODE NEEDS CLEANUP



;;(import 'SPLIT-SEQUENCE:split-sequence)

(define-modify-macro appendf (&rest args) 
   append "Append onto list")

(defun runJobDispachter ()
"This function contains the main loop of the component!"
 (loop 
  (let ((request nil)
        (tmp nil))
    (setf tmp  (wait-for-message))
    (setf request (read-from-string tmp))
    (format t "LISP GOT MESSAGE: ~s~%" request)

    (format t "=========================>>> HANDLER HIGHLEVEL  FLEET COMMAND: ~s ~%~%" request)
    ;;convert messages to forward to fleetcom to a lisp list to parse them
    ;;this madness is because of the key:value definition in the fleet com!
    (cond	
     ((typep request '(simple-array *))
      (let ((tmp nil)
           (outlist nil))
        (setf tmp (split-sequence:split-sequence #\space request))  
        (dolist (x tmp)
           (setf outlist (append outlist (last (split-sequence:split-sequence #\: x)))))
      (setf request (read-from-string (format nil "~a" outlist))))))

    (format t "request: ~s~%" request)

    (cond
     ;; ON_COMPONENT_SHUTDOWN
      ((equal (first request) 'ON_COMPONENT_SHUTDOWN)
        ;; FLEET --> INIT
        (tcl-kb-update :key '(is-a id) :value '((is-a fleet-class)(id 0)(state SHUTDOWN)))
        (format t "ON_COMPONENT_SHUTDOWN exit loop~%")

        (return-from runJobDispachter 'exit))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; H I G H L E V E L   C O M M A N D   I N T E R F A C E


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; JOB HANDLING

      ;; PushJob
      ((equal (first request) 'PushJob)
        (format t "PushJob - type: ~s jobID: ~s priority: ~s robotid: ~s ~%" (second request)(third request)(fourth request) (fifth request))
        ;;validate job and robotid
        (cond
         ;;TODO FIX THIS as THIS SHOULD BE STRINGS - VALIDATION OF THE JOB HAS TO BE DONE MORE EXTENSIVLY
         ((and (> (third request) 0) )

          (format t "Job valid!~%")


;;  PushJob TestJob 12 1 1
          (cond
            ;; PushJob TestJob JOBID PRIORITY ROBOTINOID
            ((equal (second request) 'TestJob)
              (format t "TestJob job ~s: ~%" (third request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request))
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (type ,(second request)))))

            ((equal (second request) 'DeliverFromTo)
              (multiple-value-bind (jobtype jobid jobpriority robotid from-station-id from-belt to-station-id to-belt)
                                   (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request))
              (format t "Deliver from Station: ~s Belt: ~s to Station ~s Belt ~s ~%" from-station-id from-belt to-station-id to-belt)
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,jobid) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,jobpriority)
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal robotid -1) nil robotid))
                                   (manual-assigned ,(if (equal robotid -1) nil T))
                                   (start-station ,from-station-id) (start-belt ,from-belt)
                                   (end-station ,to-station-id) (end-belt ,to-belt)
                                   (type ,jobtype)))))
            ((equal (second request) 'GotoPosition)
              (format t "Goto Position: ~s ~%" (sixth request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (goal-pose ,(sixth request))
                                   (type ,(second request)))))
              ;;TODO THIS NEEDS TO BE PLACED MORE GENERIC
;              (let ((job (tcl-kb-query :key '(is-a id) :value `((is-a job) (id ,(third request))))))
;              (send-job-to-elastic job)))


            ;; PushJob PalletizeItem JOBID PRIORITY ROBOTINOID STARTSTATION palletize-STATION DROP-OFF-STATION 
            ((equal (second request) 'PalletizeItem)
              (format t "PalletizeItem ~%")
              (multiple-value-bind (jobtype jobid jobpriority robotid start-station-id palletize-station-id drop-off-station-id)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request))
                (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,jobid) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,jobpriority)
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal robotid -1) nil robotid ))
                                   (manual-assigned ,(if (equal robotid -1) nil T))
                                   (start-station ,start-station-id) (start-station-belt 1)
                                   (palletize-station ,palletize-station-id) (palletize-station-belt 1)
                                   (drop-off-station ,drop-off-station-id) (drop-off-station-belt 1)
                                   (type ,jobtype)))))

            ;; PushJob DeliverPallet JOBID PRIORITY ROBOTINOID palletize-STATION DROP-OFF-STATION
            ((equal (second request) 'DeliverPallet)
              (format t "DeliverFullPalet ~%")
              (multiple-value-bind (jobtype jobid jobpriority robotid palletize-station-id drop-off-station-id)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request))
                (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,jobid) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,jobpriority)
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal robotid -1) nil robotid ))
                                   (manual-assigned ,(if (equal robotid -1) nil T))
                                   (palletize-station ,palletize-station-id) (palletize-station-belt 1)
                                   (drop-off-station ,drop-off-station-id) (drop-off-station-belt 1)
                                   (type ,jobtype)))))

            ;; PushJob RobotCommissioning JOBID PRIORITY ROBOTINOID COMMISSIONINGROBOT FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
            ((equal (second request) 'RobotCommissioning)
              (multiple-value-bind (jobtype jobid jobpriority robotid commissioningrobotid from-box-station-id from-box-belt to-station-id to-belt)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request) (tenth request))
              (format t "RobotCommissioning job: id ~s from Robot: ~s to Station ~s Belt ~s ~%" jobid commissioningrobotid to-station-id to-belt)
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,jobid)
                                   (type ,jobtype)
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,jobpriority)
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal robotid -1) nil robotid))
                                   (manual-assigned ,(if (equal robotid -1) nil T))
                                   (commissioning-robot ,commissioningrobotid)
                                   (start-location ,from-box-station-id) (start-belt ,from-box-belt)
                                   (end-location ,to-station-id) (end-belt ,to-belt)
                                   (commission-order ,((lambda () (setf tmp nil) (loop for (a b) on (nthcdr 10 request) by #'cddr do (push (list a b) tmp)) tmp)))))))



	     ; PushJob ProductProduction 1 3 1 10 1 1 3 1 "CADModelA"
             ; PushJob ProductProduction 2 3 1 10 1 1 3 1 "CADModelB"

             ; PushJob MPSDocking 1 0 1 DOCK 1 1

             ; PushJob MPSDocking 1 0 1 UNDOCK 1 1

            ;; PushJob ProductProduction JOBID PRIORITY ROBOTINOID MANIPULATIONROBOT FROMSTATION FROMBELT TOSTATION TOBELT PRODUCTIONDATA
            ((equal (second request) 'ProductProduction)
              (multiple-value-bind (jobtype jobid jobpriority robotid manipulationrobotid from-box-station-id from-box-belt to-station-id to-belt productiondata)
                                    (values (second request) (third request) (fourth request) (fifth request) (sixth request) (seventh request) (eighth request) (ninth request) (tenth request))
              (format t "ProductProduction job: id ~s from Robot: ~s to Station ~s Belt ~s ~%" jobid manipulationrobotid to-station-id to-belt)
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,jobid)
                                   (type ,jobtype)
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,jobpriority)
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal robotid -1) nil robotid))
                                   (manual-assigned ,(if (equal robotid -1) nil T))
                                   (manipulation-robot ,manipulationrobotid)
                                   (start-location ,from-box-station-id) (start-belt ,from-box-belt)
                                   (end-location ,to-station-id) (end-belt ,to-belt)
                                   (productiondata ,(nth 10 request))))))


            ;; PushJob FollowPerson JOBID PRIORITY ROBOTINOID
            ((equal (second request) 'FollowPerson)
              (format t "FollowPerson job ~s ~%" (third request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (type ,(second request)))))

            ;; PushJob MPSDocking JOBID PRIORITY ROBOTINOID ACTION STATIONID TOBELT
            ((equal (second request) 'MPSDocking)
              (format t "MPSDocking job ~s action: ~s ~%" (third request) (sixth request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (docking-action ,(sixth request))
                                   (docking-station-id ,(seventh request))
                                   (docking-to-belt ,(eighth request))
                                   (type ,(second request)))))

            ;; PushJob MPSLoading JOBID PRIORITY ROBOTINOID ACTION STATIONID
            ((equal (second request) 'MPSLoading)
              (format t "MPSLoading job ~s action: ~s ~%" (third request) (sixth request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (loading-action ,(sixth request))
                                   (stationid ,(seventh request))
                                   (type ,(second request)))))

            ;; PushJob BatteryChargerDocking JOBID PRIORITY ROBOTINOID ACTION
            ((equal (second request) 'BatteryChargerDocking)
              (format t "BatteryChargerDocking job ~s action: ~s ~%" (third request) (sixth request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (docking-action ,(sixth request))
                                   (type ,(second request)))))

            ;; PushJob RobotGripper JOBID PRIORITY ROBOTINOID ACTION
            ((equal (second request) 'RobotGripper)
              (format t "RobotGripper job ~s action: ~s ~%" (third request) (sixth request))
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,(third request)) 
                                   (state nil)
                                   (life-cycle-state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,(fourth request))
                                   (push-time ,(get-universal-time))
                                   (robotid ,(if (equal (fifth request) -1) nil (fifth request)))
                                   (manual-assigned ,(if (equal (fifth request) -1) nil T))
                                   (gripper-action ,(sixth request))
                                   (type ,(second request)))))
            (T
              (format t "===== ERROR UNKOWN JOB TYPE SEND! ~%"))))
          (T
            (format t "===== INVALID JOB SEND! ~%"))))


      ;; UpdateJob
      ((equal (first request) 'UpdateJob)
        (format t "UpdateJob - type: ~s jobID: ~s ~%" (second request)(third request))
        (let* ((job (tcl-kb-query :key '(is-a id) :value `((is-a job) (id ,(third request)))))
               (type (get-value job 'type))
               (life-cycle-state (get-value job 'life-cycle-state))
               (robot (get-value job 'robotid)))
                               
          (format t "UpdateJob job-id:~s robot-id:~s job-type:~s job-life-cycle-state:~s ~%" (third request) robot type life-cycle-state)

          ;;validate job and robotid
          (cond
            ((and (> (third request) 0) 
                  (not (equal life-cycle-state 'NOTSTARTED)) (not (equal life-cycle-state 'FINISHED)) (not (equal life-cycle-state 'ABORTED)) (not (equal life-cycle-state 'NOTAVAILABLE)))
              (format t "UpdateJob valid!~%")
              (send-job request robot))
                
            (T
              (format t "===== INVALID JOBUPDATE SEND! ~%")))))

      ;; EndJob
      ((equal (first request) 'EndJob)
        (format t "EndJob - jobID: ~s ~%" (second request))
        (let* ((job (tcl-kb-query :key '(is-a id) :value `((is-a job) (id ,(second request)))))
               (type (get-value job 'type))
               (life-cycle-state (get-value job 'life-cycle-state))
               (robot (get-value job 'robotid)))
                               
          (format t "EndJob job-id:~s robot-id:~s job-type:~s job-life-cycle-state:~s ~%" (second request) robot type life-cycle-state)
          ;;validate job and robotid
          (cond
            ((and (> (second request) 0) 
                  (not (equal life-cycle-state 'NOTSTARTED)) (not (equal life-cycle-state 'FINISHED)) (not (equal life-cycle-state 'ABORTED)) (not (equal life-cycle-state 'NOTAVAILABLE)))
              (format t "EndJob valid!~%")
              (send-job request robot))
                
            (T
              (format t "===== INVALID ENDJOB SEND! ~%")))))


      ;; DeleteJob
      ((equal (first request) 'DeleteJob)
        (format t "DeleteJob - jobID: ~s ~%" (second request))
        (let ((job (tcl-kb-query :key '(is-a id) :value `((is-a job)(id ,(second request))))))
          (cond
            ((null job)
              (format t "ERROR job to delete not found in KB!~%")
              (tcl-send :server 'highlevelcommand :service 'result :param 
                (format nil "DeleteJob jobid:~d failed" (second request))))
            (T
              (format t "Found job in KB!~%")
              (cond
                ((equal (get-value job 'life-cycle-state) 'NOTSTARTED)
                 (format t "Job not started yet --> remove job!~%")
                 (tcl-kb-delete :key '(is-a id) :value `((is-a job) (id ,(second request))))
                  (tcl-send :server 'highlevelcommand :service 'result :param 
                    (format nil "DeleteJob jobid:~d success" (second request))))
                (T
                  (format t "Job has already been started or is finished --> report error~%")
                  (tcl-send :server 'highlevelcommand :service 'result :param 
                    (format nil "DeleteJob jobid:~d failed" (second request)))))))))

      ;; DeleteNotRunningJobs
      ((equal (first request) 'DeleteNotRunningJobs)
        (format t "DeleteNotRunningJobs~%")
        (tcl-kb-delete :key '(is-a life-cycle-state) :value `((is-a job)  (life-cycle-state NOTAVAILABLE)))
        (tcl-kb-delete :key '(is-a life-cycle-state) :value `((is-a job)  (life-cycle-state FINISHED)))
        (tcl-kb-delete :key '(is-a life-cycle-state) :value `((is-a job)  (life-cycle-state ABORTED)))
        (tcl-kb-delete :key '(is-a life-cycle-state) :value `((is-a job)  (life-cycle-state NOTSTARTED))))


      ;; DeleteAllJobs
      ; for intern usage only!
      ((equal (first request) 'DeleteAllJobs)
        (format t "DeleteAllJobs~%")
        (tcl-kb-delete :key '(is-a) :value `((is-a job)))
        (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
          (dolist (robot robot-list)
            (send-job '(DeleteAllJobs) (get-value robot 'name)))))

;; JOB HANDLING
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ROBOT STATE HANDLING

      ;; SetOperationMode
      ((equal (first request) 'SetOperationMode)
        (format t "SetOperationMode robotid: ~s mode: ~s ~%" (second request)(third request))
        (tcl-kb-update :key '(is-a name) :value `((is-a robot) 
                                                (name ,(second request)) (mode ,(third request))))
        (send-job `(SetOperationMode ,(second request) ,(third request)) (second request)))

      ;; ErrorAcknowledge
      ((equal (first request) 'AbortJobAndClearError)
        (format t "AbortJobAndClearError robotinoid: ~s ~%" (second request))
        (send-job `(AbortJobAndClearError ,(second request)) (second request)))

      ;; ClearError
      ((equal (first request) 'ClearError)
        (format t "ClearError robotinoid: ~s ~%" (second request))
        (send-job `(ClearError ,(second request)) (second request)))

      ;; ManualAcknowledge
      ((equal (first request) 'ManualAcknowledge)
        (format t "ManualAcknowledge robotinoid: ~s ~%" (second request))
        (send-job `(ManualAcknowledge ,(second request)) (second request)))

      ;;ShutdownRobot - active command from outside
      ((equal (first request) 'ShutdownRobot)
        (format t "ShutdownRobot robotinoid: ~s ~%" (second request))
        (let* ((robots-in-fleet (get-value (tcl-kb-query :key '(is-a id) :value '((is-a job)(id 0))) 'robots)))
             (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (robots ,(remove (second request) robots-in-fleet)))))

        ;; abort job if running
        (let* ((robot (tcl-kb-query :key '(is-a name) :value `((is-a robot)(name ,(second request)))))
               (running-job (get-value robot 'performs-task)))
          (if (null running-job)
            (tcl-kb-update :key '(is-a id) :value `((is-a job) (id ,running-job) (life-cycle-state ABORTED))))

          (tcl-param :server 'highlevelcommand :slot 'DOMAINROBOTFLEET.FLEETMANAGERPARAMETER.RMROBOT :value `(,(second request)))
          (tcl-param :server 'robotinorpcbridge :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.RMROBOT :value `(,(second request)))

          (tcl-param :server 'pathnavigationserver :slot 'COMMNAVIGATIONOBJECTS.CORRIDORNAVIGATIONSERVERPARAMS.REL_ALL_NODES :value (second request)))

          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot))))
                (robot-master-id nil))
            (loop for robot in robot-list
              with res = -1
              when (equal (get-value robot 'robotip) *MASTERIP*)
                do (setf res (get-value robot 'name))
                finally (setf robot-master-id res))

          (format t "Master robot id ~a~%" robot-master-id)


          (cond
            ((equal robot-master-id (second request))
            ;; FLEET --> SHUTDOWN
            (tcl-kb-update :key '(is-a id) :value '((is-a fleet-class)(id 0)(state SHUTDOWN)))
              ;first forward shutdown to all robots!
              (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
                (dolist (robot robot-list)
                  (format t " ShutdownRobot robots:-~%")
                  (format t "  name              : ~s ~%" (get-value robot 'name))
                  (send-job '(ShutdownRobot) (get-value robot 'name))))
              ;shutdown own components   
              (tcl-state :server 'symbolicplanner :state "Shutdown")
              (tcl-state :server 'highlevelcommand :state "Shutdown")
              (tcl-state :server 'knowledgebase :state "Shutdown")
              (tcl-state :server 'pathnavigationserver :state "Shutdown")
              (tcl-state :server 'fileprovider :state "Shutdown")
              (tcl-state :server 'robotinorpcbridge :state "Shutdown")
              (tcl-state :server 'mapper :state "Shutdown")

              (format t "Shutdown of JobDispatcher DONE -->QUIT~%")
              (exit))
            (T
              (tcl-kb-delete :key '(is-a name) :value `((is-a robot)(name ,(second request))))
              (send-job request (second request))))))


;; ROBOT STATE HANDLING
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; OTHER COMMANDS

      ;; SetRobotPose
      ((equal (first request) 'SetRobotPose)
        (format t "SetRobotPose robotid: ~s ~%" (second request))
        (send-job `(SetRobotPose ,(second request) ,(third request) ,(fourth request) ,(fifth request)) (second request)))

      ;; PushCommand
      ((equal (first request) 'PushCommand )
        (format t "PushCommand ~s robotinoid: ~s ~%" (rest (rest request)) (second request))
        (send-job (rest (rest request)) (second request)))

      ;; EndTask
      ((equal (first request) 'EndTask)
	(format t "EndTask id: ~s~%" (second request))
        (cond
          ((not (equal (second request) nil))
           (format t " Forward End task postion to robot:~s~%" (second request))
           (send-job request (second request)))
        (T
          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
            (dolist (robot robot-list)
              (format t " Forward End task postion to robots:-~%")
              (format t "  name              : ~s ~%" (get-value robot 'name))
              (send-job request (get-value robot 'name)))))))

      ;; PauseRobot
      ((equal (first request) 'PauseRobot)
	(format t "PauseRobot id: ~s~%" (second request))
        (cond
          ((not (equal (second request) nil))
           (format t " Forward PauseRobot to robot:~s~%" (second request))
           (send-job request (second request)))
        (T
          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
            (dolist (robot robot-list)
              (format t " Forward PauseRobot to robots:-~%")
              (format t "  name              : ~s ~%" (get-value robot 'name))
              (send-job request (get-value robot 'name)))))))

      ;; ContinueRobot
      ((equal (first request) 'ContinueRobot)
	(format t "ContinueRobot id: ~s~%" (second request))
        (cond
          ((not (equal (second request) nil))
           (format t " Forward ContinueRobot to robot:~s~%" (second request))
           (send-job request (second request)))
        (T
          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
            (dolist (robot robot-list)
              (format t " Forward ContinueRobot to robots:-~%")
              (format t "  name              : ~s ~%" (get-value robot 'name))
              (send-job request (get-value robot 'name)))))))

      ;; GotoPosition
      ((equal (first request) 'GotoPosition)
        (format t "GotoPosition robotinoid: ~s position: ~s ~%" (second request)(third request))
        (send-job request (second request)))

      ;; GetRobotIDMasterComponentsRunOn
      ((equal (first request) 'GetRobotIDMasterComponentsRunOn)
        (format t "GetRobotIDMasterComponentsRunOn ip:~s~%" *MASTERIP*)
        (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
          (loop for robot in robot-list
            with res = -1
            when (equal (get-value robot 'robotip) *MASTERIP*)
              do (setf res (get-value robot 'name))
              finally (tcl-send :server 'highlevelcommand :service 'result :param (format nil "RobotIDMasterComponentsRunOn ~s" res)))))


;; OTHER COMMANDS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; PATHS NAV HANDLING

      ;; TODO this is done via the robots, could be done directly accessing the PathNavigationServer
      ;; ClearAllPathNetworkNodes
      ((equal (first request) 'ClearAllPathNetworkNodes)
        (format t "ClearAllPathNetworkNodes robotinoid: ~s ~%" (second request))
        (send-job `(ClearAllPathNetworkNodes ,(second request)) (second request)))

      ;; AckClearAllPathNetworkNodes
      ((equal (first request) 'AckClearAllPathNetworkNodes)
        (format t "GOT AckClearAllPathNetworkNodes --> FORWARD")
        (tcl-send :server 'highlevelcommand :service 'result :param 
          (format nil "AckClearAllPathNetworkNodes robotinoid:~d " (second request))))

      ;; ReloadDefaultPaths
      ((equal (first request) 'ReloadDefaultPaths)
        (format t "ReloadDefaultPaths THIS SOULD ONLY BE CALLED WHILE NONE!! OF THE ROBOTS IS NOT MOVEING~%")
        (tcl-param :server 'pathnavigationserver :slot 'COMMNAVIGATIONOBJECTS.CORRIDORNAVIGATIONSERVERPARAMS.LOADPATHLIST :value "data_master/maps/default/navigation-paths.xml")
        '(SUCCESS()))

;; PATHS NAV HANDLING
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; STATION HANDLING

      ;; ReplaceStations
      ((equal (first request) 'ReplaceStations)
        (cond
         ((< (length request) 2)
           (format t "Stations list is emtpy --> delete all stations~%")
           (tcl-kb-delete :key '(is-a) :value `((is-a station))))
         (T
                 ;(handler-case
                 ;  (split-sequence:SPLIT-SEQUENCE 'Position request :start 2 )
                 ;  (SB-KERNEL:BOUNDING-INDICES-BAD-ERROR () 
                 ;    (format t "Error in SPLIT-SEQUENCE --> IGNORE~%")))
           (let ((station-list (split-sequence:SPLIT-SEQUENCE 'Station request :start 2 ))
                (kb-update-list nil))
            (dolist (station station-list)
              (multiple-value-bind (id x y yaw type approach-location)
                (apply #'values station)
                (format t "Station ID: ~s  x: ~s  y: ~s phi: ~s type: ~s app.Loc: ~s ~%" id x y yaw type approach-location)
                (let ( (station-pose `(,x ,y 0.0 ,(* (/ yaw 180) pi) 0.0 0.0)))
                  (push (list '(is-a id)
                         `( 
                           (is-a station)
                           (id ,id)
                           (type ,type)
                           (approach-location, approach-location)
                           (pose ,station-pose)))
                           kb-update-list))))

             (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value '((is-a station))))))

        (format t "ReplaceStations bevor write!~%")
        (show-stations)

        ;; save stations to FILE
        (save-stations-from-kb-to-file))

      ;; DeleteAllStations
      ((equal (first request) 'DeleteAllStations)
	   (format t "DeleteAllStations ~%")
        (tcl-kb-delete 
          :key '(is-a) 
          :value `( 
                    (is-a station)))

        ;; save stations to FILE
        (save-stations-from-kb-to-file))

      ;; NewStationDetected
      ((equal (first request) 'NewStationDetected)
        (multiple-value-bind (id x y z yaw pitch roll station-type)
          (apply #'values (rest request))
          (format t "NewStationDetected Tag ~a station-type:~s x:~a y:~a z:~a yaw:~a pitch: ~a roll: ~a ~%" id station-type x y z yaw pitch roll)
          (cond
            ((equal station-type 'small-mps)
              (let* ((marker-pose          `(,x ,y ,z      ,(* (/ yaw 180) pi) ,(* (/ pitch 180) pi) ,(* (/ roll 180) pi)))
                     (marker-to-robot-pose `(0.0 0.0 0.0   ,(/ pi 2) ,(/ pi 2) 0.0))
                     (small-mps-offset     `(0.175 -0.525 0.0   ,(/ pi 2) 0.0 0.0))
                     (station-pose-offset  `(-0.7 0.0 0.0  0.0 0.0 0.0))
                     (station-pose (composePoses3D (composePoses3D marker-pose marker-to-robot-pose) small-mps-offset))
                     (station-approach-pose (composePoses3D station-pose station-pose-offset)))
                (format t "calculated station-pose (x y z yaw pitch roll): ~a~%" station-pose)
                (format t "calculated station-approach-pose (x y z yaw pitch roll): ~a~%" station-approach-pose)
            ;(tcl-kb-update :key '(is-a name) :value`( 
            ;                (is-a location)
            ;                (name ,id)
            ;                (type pose)
            ;                (approach-type (path-nav))
            ;                (approach-region-pose (,(round(*(first station-approach-pose) 1000)),(round (* (second station-approach-pose) 1000)) 0))
            ;                (approach-region-dist 75)
            ;                (approach-exact-pose (,(round (*(first station-approach-pose) 1000)),(round (*(second station-approach-pose) 1000)) 0))
            ;                (approach-exact-dist 100)
            ;                (approach-exact-safetycl 0)
            ;                (orientation-region (angle-absolute ,(round (/ (* (fourth station-approach-pose) 180) pi))))
            ;                (orientation-exact (angle-absolute ,(round (/ (* (fourth station-approach-pose) 180) pi))))
            ;                ;(backward-dist ,(sixth request))
            ;                ;(backward-rotation ,(seventh request))
            ;                (belt-count 1)
            ;                (station-distance 0.5)
            ;                (parking-state free)))

                 (tcl-kb-update :key '(is-a id) :value `(
                              (is-a station)
                              (id ,id)
                              (pose ,station-pose)
                              ;(approach-pose ,station-approach-pose)
                              (approach-location -1)
                              (type ,station-type)))))
            (T
              (format t "ERROR unkown startion type detected: ~a ~%" station-type)))))

;; STATION HANDLING
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; POSITION HANDLING

      ;; ReplacePositions
      ((equal (first request) 'ReplacePositions)
        (cond
         ((< (length request) 2)
           (format t "Postion list is emtpy --> delete all positions~%")
           (tcl-kb-delete :key '(is-a) :value `((is-a location))))
         (T
                 ;(handler-case
                 ;  (split-sequence:SPLIT-SEQUENCE 'Position request :start 2 )
                 ;  (SB-KERNEL:BOUNDING-INDICES-BAD-ERROR () 
                 ;    (format t "Error in SPLIT-SEQUENCE --> IGNORE~%")))
           (let ((positions-list (split-sequence:SPLIT-SEQUENCE 'Position request :start 2 ))
                (kb-update-list nil))
            (dolist (pose positions-list)
              (multiple-value-bind (id x y yaw type)
                (apply #'values pose)
                (format t "Position ID: ~s  x: ~s  y: ~s phi: ~s type:~s ~%" id x y yaw type)
                  (push (list '(is-a name)
                           `( 
                            (is-a location)
                            (name ,id)
                            (type ,type)
                            (approach-type (path-nav))
                            (approach-region-pose (,(round(* x 1000)),(round (* y 1000)) 0))
                            (approach-region-dist 75)
                            (approach-exact-pose (,(round (* x 1000)),(round (* y 1000)) 0))
                            (approach-exact-dist 100)
                            (approach-exact-safetycl 0)
                            (orientation-region (angle-absolute ,(round yaw)))
                            (orientation-exact (angle-absolute ,(round yaw)))
                            ;(backward-dist ,(sixth request))
                            ;(backward-rotation ,(seventh request))
                            (parking-state free))) kb-update-list)))
                           
             (tcl-kb-update-batch :updates-list kb-update-list :delete-key '(is-a) :delete-value '((is-a location))))))

        ;;save postions from KB to file
        (save-positions-from-kb-to-file)
        (send-command-to-all-robots '(ClearRobotsCurrentSymbolicPosition)))

;MCL
        ;(let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot))))
        ;      (station-list (tcl-kb-query-all :key '(is-a) :value `((is-a station)))))
        ;  (dolist (robot robot-list)
        ;    (format t " Forward Replace postion to robots:-~%")
        ;    (format t "  name              : ~s ~%" (get-value robot 'name))
        ;    (send-job '(DeleteAllStations) (get-value robot 'name))
        ;    (dolist (station station-list)
        ;      (format t " Forward station to robots:-~%")
        ;        (format t "  name              : ~s ~%" (get-value robot 'name))
        ;        (send-job `(UpdateStation ,(get-value station 'id) ,(get-value station 'type) 
        ;                                  ,(get-value station 'belt-count) ,(get-value station 'docking-type) 
        ;                                  ,(get-value station 'pose) ,(get-value station 'approach-pose) 
        ;                                  ,(get-value station 'approach-location)) (get-value robot 'name)))
        ;    (send-job request (get-value robot 'name)))))


      ;; TeachPosition
      ((equal (first request) 'TeachPosition)
        (let ((positions-list (split-sequence:SPLIT-SEQUENCE 'TeachPosition request :start 1 ))
              (kb-update-list nil))
          (dolist (pose positions-list)
	    (multiple-value-bind (id x y yaw type)
                (apply #'values pose)
                (format t "Position ID: ~s  x: ~s  y: ~s phi: ~s type:~s ~%" id x y yaw type)
                 (push (list '(is-a name) 
                           `( 
                            (is-a location)
                            (name ,id)
                            (type ,type)
                            (approach-type (path-nav))
                            (approach-region-pose (,(round(* x 1000)),(round (* y 1000)) 0))
                            (approach-region-dist 75)
                            (approach-exact-pose (,(round (* x 1000)),(round (* y 1000)) 0))
                            (approach-exact-dist 100)
                            (approach-exact-safetycl 0)
                            (orientation-region (angle-absolute ,(round yaw)))
                            (orientation-exact (angle-absolute ,(round yaw)))
                            ;(backward-dist ,(sixth request))
                            ;(backward-rotation ,(seventh request))
                            (parking-state free))) kb-update-list)))
           (tcl-kb-update-batch :updates-list kb-update-list))

        ;;save postions from KB to file
        (save-positions-from-kb-to-file))

      ;; TeachCurrentPosition
      ((equal (first request) 'TeachCurrentPosition)
        (format t "TeachCurrentPosition robotinoid: ~s position: ~s ~%" (second request)(third request))
        (send-job request (second request)))

      ;; DeleteAllPositions
      ((equal (first request) 'DeleteAllPositions)
	   (format t "DeleteAllPositions ~%")
        (tcl-kb-delete 
          :key '(is-a) 
          :value `( 
                    (is-a location)))

        ;;save postions from KB to file
        (save-positions-from-kb-to-file)

        (send-command-to-all-robots '(ClearRobotsCurrentSymbolicPosition)))


      ;; DeletePosition
      ((equal (first request) 'DeletePosition)
	   (format t "DeletePosition ID: ~s  ~%" (second request))
        (tcl-kb-delete 
          :key '(is-a name) 
          :value `( 
                    (is-a location)
                    (name ,(second request))))


        ;;save postions from KB to file
        (save-positions-from-kb-to-file)

        (send-command-to-all-robots '(ClearRobotsCurrentSymbolicPosition)))

;; POSITION HANDLING
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; MAPPING (SLAM)

      ;; StartMapping
      ((equal (first request) 'StartMapping)
        (format t "StartMapping robotinoid: ~s mapname: ~s ~%" (second request)(third request))
        (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (state MAPPING) (robot-mapping ,(second request))))

        (format t "DELETE ALL POSTIONS AND STATIONS IN MASTER MEMORY~%")
        (tcl-kb-delete :key '(is-a) :value `((is-a location)))
        (tcl-kb-delete :key '(is-a) :value `((is-a station)))

        (send-job request (second request))
        ;;fetch map data from slam
        (tcl-state :server 'robotinorpcbridge :state "Deactivated")
        (tcl-wiring-connect :clientComp "master.SmartRobotinoMasterRPCBridge" :wiringName "setMapClient"
                               :serverComp (format nil "~a.ComponentGMapping" (second request)) :serverService "newestMapPushServer")
        (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.SET_MAPPING_ROBOT_ID :value (second request))
        (tcl-param :server 'robotinorpcbridge  :slot 'COMMIT)
        (tcl-state :server 'robotinorpcbridge :state "slamMap"))

      ;; AbortMapping
      ((equal (first request) 'AbortMapping)
        (format t "AbortMapping robotinoid: ~s ~%" (second request))
        (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (state OPERATIONAL) (robot-mapping nil)))
        (cond
          ((not (equal (second request) nil))
           (format t " Forward AbortMapping to robot:~s~%" (second request))
           (send-job request (second request)))
        (T
          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
            (dolist (robot robot-list)
              (format t " Forward AbortMapping to robots:-~%")
              (format t "  name              : ~s ~%" (get-value robot 'name))
              (send-job request (get-value robot 'name))))))

        (format t "DELETE ALL POSTIONS AND STATIONS IN MASTER MEMORY~%")
        (tcl-kb-delete :key '(is-a) :value `((is-a location)))
        (tcl-kb-delete :key '(is-a) :value `((is-a station)))

        (format t "LOAD POSTIONS FROM FILE~%")
        (load-positions-from-file-to-KB)

        (format t "LOAD STATIONS FROM FILE~%")
        (load-stations-from-file-to-KB)

        (tcl-param :server 'mapper :slot 'COMMNAVIGATIONOBJECTS.MAPPERPARAMS.LTMLOADYAML :value "navigation-map")
;        (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.SET_LOCALIZATION_TYP :value `(LOCALIZATION -1))
        (tcl-state :server 'robotinorpcbridge :state "Deactivated")
        (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.REFRESH_STATIC_MAP)
        (tcl-state :server 'robotinorpcbridge :state "staticMap")
        (tcl-wiring-disconnect :clientComp "master.SmartRobotinoMasterRPCBridge" :wiringName "setMapClient"))

      ;; SaveMap
      ((equal (first request) 'SaveMap)
        (format t "SaveMap robotinoid: ~s ~%" (second request))
        (send-job request (second request)))

      ;; StopMapping
      ((equal (first request) 'StopMapping)
        (format t "StopMapping robotinoid: ~s ~%" (second request))
        (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (state OPERATIONAL) (robot-mapping nil)))
        (send-job request (second request))
        (tcl-state :server 'robotinorpcbridge :state "Deactivated")
        (tcl-wiring-disconnect :clientComp "master.SmartRobotinoMasterRPCBridge" :wiringName "setMapClient"))

      ;; NewMapSaved --> set as new default and load on all robots!
      ((equal (first request) 'NewMapSaved)
 	(format t "NewMapSaved name: ~s ~%" (second request))
        (let* ((defaultMapPath (format NIL "~a/data_master/maps/default" (sb-ext:posix-getenv "SMART_ROOT_ACE")))
               (newMapPath (format NIL "~a/data_master/maps/~a" (sb-ext:posix-getenv "SMART_ROOT_ACE") (second request)))
               (mapping-robot-id (third request) )
               (mapping-robot-pose-x (fourth request))
               (mapping-robot-pose-y (fifth request))
               (mapping-robot-pose-phi (sixth request)))
              
          (cond
           ;;default is there
           ((not (equal (probe-file defaultMapPath) nil))
             (sb-ext:run-program "/usr/bin/unlink" `(,defaultMapPath) :wait T))
           ;;default is not there
           (T
             (format t "WARNING no default map defined so far --> set one!~%")))
          (format t "newMapPath: ~a ~%" newMapPath)
          (sb-ext:run-program "/bin/ln" `("-s" ,newMapPath ,defaultMapPath) :wait T)

          (format t "Create new locations.lisp FILE~%")
          (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/positions.lisp") :if-does-not-exist :create) 
           nil)
          (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/stations.lisp") :if-does-not-exist :create) 
           nil)

          ;; save-positions-from-kb-to-file
          (save-positions-from-kb-to-file)
          ;; save stations to FILE
          (save-stations-from-kb-to-file)
 
          (format t "LOAD PATHS FROM FILE~%")
          (tcl-param :server 'pathnavigationserver :slot 'COMMNAVIGATIONOBJECTS.CORRIDORNAVIGATIONSERVERPARAMS.LOADPATHLIST :value "data_master/maps/default/navigation-paths.xml")

          (format t "Reload map in Mapper and trigger RPC Bridge~%")
          (tcl-param :server 'mapper :slot 'COMMNAVIGATIONOBJECTS.MAPPERPARAMS.LTMLOADYAML :value "navigation-map")
        ;;  (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.SET_LOCALIZATION_TYP :value `(LOCALIZATION -1))
          (tcl-state :server 'robotinorpcbridge :state "Deactivated")
          (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.REFRESH_STATIC_MAP)
          (tcl-state :server 'robotinorpcbridge :state "staticMap")

          ;(format t "DELETE ALL POSTIONS IN MASTER MEMORY~%")
          ;(tcl-kb-delete :key '(is-a) :value `((is-a location)))
          (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
            (dolist (robot robot-list)
              (format t " ReloadDefault map to robots:-~%")
              (format t "  name              : ~s ~%" (get-value robot 'name))
              (send-job '(ReloadDefaultMap) (get-value robot 'name))
              (if (equal (get-value robot 'name) mapping-robot-id)
                (send-job `(SetRobotPose ,(get-value robot 'name) ,mapping-robot-pose-x ,mapping-robot-pose-y ,mapping-robot-pose-phi) (get-value robot 'name)))))))
;              (send-job '(DeleteAllPositions) (get-value robot 'name)))))

      ;; ReloadDefaultMap
      ((equal (first request) 'ReloadDefaultMap)
        (format t "ReloadDefaultMap ~%")
        (tcl-param :server 'mapper :slot 'COMMNAVIGATIONOBJECTS.MAPPERPARAMS.LTMLOADYAML :value "navigation-map")
      ;;  (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.SET_LOCALIZATION_TYP :value `(LOCALIZATION -1))
        (tcl-state :server 'robotinorpcbridge :state "Deactivated")
        (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.REFRESH_STATIC_MAP)
        (tcl-state :server 'robotinorpcbridge :state "staticMap")
        (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
          (dolist (robot robot-list)
            (format t " ReloadDefaultMap robots:-~%")
            (format t "  name              : ~s ~%" (get-value robot 'name))
            (send-job '(ReloadDefaultMap) (get-value robot 'name))))


        (format t "DELETE ALL POSTIONS IN MASTER MEMORY~%")
        (tcl-kb-delete :key '(is-a) :value `((is-a location)))
        (format t "DELETE ALL STATIONS IN MASTER MEMORY~%")
        (tcl-kb-delete :key '(is-a) :value `((is-a station)))

        (format t "LOAD POSTIONS FROM FILE~%")
        (load-positions-from-file-to-KB)

        (format t "LOAD STATIONS FROM FILE~%")
        (load-stations-from-file-to-KB)

       (format t "LOAD PATHS FROM FILE~%")
       ;; this needs to be a passive handler
       (tcl-param :server 'pathnavigationserver :slot 'COMMNAVIGATIONOBJECTS.CORRIDORNAVIGATIONSERVERPARAMS.LOADPATHLIST :value "data_master/maps/default/navigation-paths.xml"))

;; MAPPING (SLAM)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



      ;; GetAllRobotinoID --> handeled by FleetCom
      ;; GetAllPosition --> handeled by FleetCom
      ;; GetAllStation --> handeled by FleetCom
      ;; GetAllStationTypes --> handeled by FleetCom
      ;; GetRobotInfo --> handeled by FleetCom 
      ;; GetJobInfo --> handeled by FleetCom
      ;; GetJobError --> handeled by FleetCom
      

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; R O B O T I N O   I N T E R N A L   I N T E R F A C E

      ;; RoboJobTimingInfo
       ; RoboJobTimingInfo robotid timing
      ((equal (first request) 'RoboJobTimingInfo)
        (dolist (info (third request))
         ;; (send-job-timings info (second request))
          (tcl-kb-update :key '(is-a type robotid) :value `(
                                   (is-a job-timing)
                                   (robotid ,(second request))
                                   (type ,(first info))
                                   (timing ,(second info))))))
      


;        (ROBOJOBTIMINGINFO 6
;                              ((TCJ-DELIVER-FROM-TO-MPS-STATION NIL)
;                               (TCJ-GOTO-POSITION
;                                ((3766988865 3766988880)
;                                 (3766988948 3766988963)
;                                 (3766989001 3766989009)
;                                 (3766989013 3766989028)
;                                 (3766989032 3766989039)
;                                 (3766989046 3766989060)))))



      ;; SystemState
       ; SystemState name performs-task state box-loaded is-docked
      ((equal (first request) 'SystemState)
        (format t "Message of SytemState:~s ~%" request) 
        (tcl-kb-update 
         :key '(is-a name) 
         :value `(
            (is-a robot)
            (name ,(second request))
            (base-component ,(third request))
            (performs-task ,(fourth request))
            (state ,(fifth request))
            (mode ,(sixth request))
            (is-docked ,(seventh request))
            (box-loaded ,(eighth request))
            (laser-component ,(ninth request))
            (fleet-type ,(tenth request))
            (sub-state ,(nth 10 request))
            (current-symbolic-position ,(nth 11 request))
            (is-job-on-time ,(nth 12 request)) )))

      ;; WorldStateUpdate
      ;((equal (first request) 'WorldStateUpdate)
      ;  (format t "Message of WorldStateUpdate:~s ~%" request)
      ;  (cond
      ;    ((equal (second request) 'LocationParkingStateUpdate)
      ;      (tcl-kb-update :key '(is-a name) :value `((is-a location) (name ,(third request)) (parking-state ,(fourth request)))))))


; UpdateParkingAndRobotState 6 2 occupied
; UpdateParkingAndRobotState 6 2 FREE


      ((equal (first request) 'UpdateParkingAndRobotState)
        (format t "Message of UpdateParkingAndRobotState:~s ~%" request)
        (cond
          ((equal (fourth request) 'occupied)
           (let* ((robotid (second request))
                 (new-parking-pose (third request))
                 (old-parking-pose (second (get-value (tcl-kb-query :key '(is-a name) :value `((is-a robot) (name ,robotid))) 'is-parked))))

            (format t "new-parking-pose: ~s old-parking-pose: ~s ~%" new-parking-pose old-parking-pose)
            (cond
              ((equal new-parking-pose old-parking-pose)
                (format t "new-parking-pose  =  old-parking-pose --> IGNORE~%"))
              (T
                
                (if (not (null old-parking-pose))
                  (progn (format t "new-parking-pose  !=  old-parking-pose --> FREE OLD + QCCUPY NEW + SET ROBOT~%")
                          (tcl-kb-update :key '(is-a name) :value `((is-a location) (name ,old-parking-pose) (parking-state free))))
                  (format t "new-parking-pose  !=  old-parking-pose --> QCCUPY NEW~% + SET ROBOT"))
                (tcl-kb-update :key '(is-a name) :value `((is-a location) (name ,new-parking-pose) (parking-state (T ,robotid))))
                (tcl-kb-update :key '(is-a name) :value `((is-a robot) (name ,robotid) (is-parked (T ,new-parking-pose))))))))

           ((equal (fourth request) 'free)
             (let* ((robotid (second request))
                   (old-parking-pose (second (get-value (tcl-kb-query :key '(is-a name) :value `((is-a robot) (name ,robotid))) 'is-parked))))
               (format t "FREE old-parking-pose: ~s ~%" old-parking-pose)
               (if (not (null old-parking-pose))
                 (progn (format t "old-parking-pose != NULL --> FREE OLD + SET ROBOT~%")
                        (tcl-kb-update :key '(is-a name) :value `((is-a location) (name ,old-parking-pose) (parking-state free))))
                 (format t "old-parking-pose == Nill --> SET ROBOT~%"))
                 (tcl-kb-update :key '(is-a name) :value `((is-a robot) (name ,robotid) (is-parked nil)))))))
               
           
 

      ;; JobInfo
       ; JobInfo robotinoid:10 jobid:10 state:STARTED life-cycle-state:RUNNING start-time: end-time:
      ((equal (first request) 'JobInfo)
        (format t "GOT JOB INFO--> UPDATE KB and FORWARD~%")
        (tcl-kb-update :key '(is-a id) :value `((is-a job) 
                                                (id ,(third request)) (robotid ,(second request)) (state ,(fourth request)) 
                                                (life-cycle-state ,(fifth request)) 
                                                (start-time ,(sixth request)) (end-time ,(seventh request))))

        (let ((job (tcl-kb-query :key '(is-a id) :value `((is-a job)(id ,(third request))))))
          ;(cond
          ;(tcl-kb-update :key '(is-a robotid) :value `((is-a robot) (robotid ,(get-value job 'name)) 
          ;                                                   (performs-task nil) (state IDLE)))
;          (send-job-to-elastic job)
          (send-job-state-change-to-fleet-com job)
          ))

      ;; JobError
       ; JobError robotinoid:10 jobid:10 error:MESSAGE
      ((equal (first request) 'JobError)
        (format t "GOT JOB ERROR--> UPDATE KB and FORWARD~%")
        (tcl-kb-update :key '(is-a id) :value `((is-a job) 
                                                (id ,(third request)) (robotid ,(second request)) (error-state ,(fourth request))))
        (let ((job (tcl-kb-query :key '(is-a id) :value `((is-a job)(id ,(third request))))))
          ;(cond
          ;(tcl-kb-update :key '(is-a robotid) :value `((is-a robot) (robotid ,(get-value job 'name)) 
          ;                                                   (performs-task nil) (state IDLE)))
;          (send-job-to-elastic job)
          (send-job-error-change-to-fleet-com job)
          ))

      ;; CommandInfo
       ; CommandInfo robotinoid:10 state:"Error Message"
      ((equal (first request) 'CommandInfo)
        (format t "GOT COMMAND INFO --> FORWARD~%")
        (send-command-info-to-fleet-com request))

      ;; GetManualAcknowledge
       ; GetManualAcknowledge 10
      ((equal (first request) 'GetManualAcknowledge)
        (format t "GOT GetManualAcknowledge --> FORWARD~%")
        (tcl-send :server 'highlevelcommand :service 'result :param 
          (format nil "GetManualAcknowledge robotinoid:~d " (second request))))

   
      ;; AddRequest
      ((equal (first request) 'AddRequest)
       (format t "AddRequest from robot: ~s type: ~s ~%" (second request) (third request))
       (let ((robots-in-fleet (get-value (tcl-kb-query :key '(is-a id) :value '((is-a job)(id 0))) 'robots)))
             (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (robots ,(push (second request) robots-in-fleet)))))
        (tcl-kb-update 
         :key '(is-a name) 
         :value `(
            (is-a robot)
            (name ,(second request))
            (state ADDREQUEST)
            (fleet-type ,(third request)))))


      ;; AddRobot
      ((equal (first request) 'AddRobot)
        (format t "Message AddRobot:~s TYPE:~s ~%" request (tenth request))
        (tcl-kb-update 
         :key '(is-a name) 
         :value `(
            (is-a robot)
            (name ,(second request))
            (base-component ,(third request))
            (performs-task ,(fourth request))
            (state ,(fifth request))
            (mode ,(sixth request))
            (is-docked ,(seventh request))
            (box-loaded ,(eighth request))
            (laser-component ,(ninth request))
            (fleet-type ,(tenth request))
            (path-nav-component ,(nth 10 request))
            (robotip ,(nth 11 request))))

            (tcl-param :server 'highlevelcommand :slot 'DOMAINROBOTFLEET.FLEETMANAGERPARAMETER.ADDROBOT :value `( ,(second request) ,(first (third request)) ,(second (third request)) ,(third (third request)) 
                                                                                  ,(first (ninth request)) ,(second (ninth request)) ,(first (nth 10 request)) ,(second (nth 10 request))))
            (tcl-param :server 'robotinorpcbridge :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.ADDROBOT 
                                 :value `( ,(second request) ,(first (third request)) ,(second (third request)) ,(third (third request)) 
                                                                                  ,(first (ninth request)) ,(second (ninth request)) ,(first (nth 10 request)) ,(second (nth 10 request))))
    
        (send-job '(AddRobotCompleted)(second request)))
        


      ;; RemoveRobot - slave init shutdown, send by slave to remove itself from fleet
      ((equal (first request) 'RemoveRobot)
        (format t "Message RemoveRobot~s: ~%" request)

        (let* ((robots-in-fleet (get-value (tcl-kb-query :key '(is-a id) :value '((is-a job)(id 0))) 'robots)))
             (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) (robots ,(remove (second request) robots-in-fleet)))))

        ;; abort job if running
        (let* ((robot (tcl-kb-query :key '(is-a name) :value `((is-a robot)(name ,(second request)))))
               (running-job (get-value robot 'performs-task)))
          (if (null running-job)
            (tcl-kb-update :key '(is-a id) :value `((is-a job) (id ,running-job) (life-cycle-state ABORTED)))))


        ;; delete robot from KB
        (tcl-kb-delete
         :key '(is-a name) 
         :value `(
            (is-a robot)
            (name ,(second request))))

        (tcl-param :server 'pathnavigationserver :slot 'COMMNAVIGATIONOBJECTS.CORRIDORNAVIGATIONSERVERPARAMS.REL_ALL_NODES :value (second request))

        (tcl-param :server 'highlevelcommand :slot 'DOMAINROBOTFLEET.FLEETMANAGERPARAMETER.RMROBOT :value `(,(second request)))
        (tcl-param :server 'robotinorpcbridge :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.RMROBOT :value `(,(second request))))


;;TODO check is this is working!
      ;; RestartSlaveRobots
      ((equal (first request) 'RestartSlaveRobots)
	(format t "RestartSlaveRobots ~%")
        (let ((robot-list (tcl-kb-query-all :key '(is-a fleet-type) :value '((is-a robot)(fleet-type "slave")))))
          (dolist (robot robot-list)
            (format t " Restart robots:-~%")
            (format t "  name              : ~s ~%" (get-value robot 'name))
            (send-job '(GlobalLocalization) (get-value robot 'name))
            (send-job '(RestartRobot) (get-value robot 'name)))))

      ;; exit
      ((equal (first request) 'EXIT)
                (return))
      ;; quit
      ((equal (first request) 'quit)
                (exit))
		
      (T
        (format t "NOT HANDELED COMMAND SEND TO JobDispatcher: ~s --> forward to MASTER (local) SEQUENCER~%" (first request))
        (format t "!!!! ERROR!!!!! THIS IS NOT WORKING ANY MORE !!!! ERROR!!!!!~%"))
        ;(let ((master-robot (tcl-kb-query :key '(is-a fleet-type) :value '((is-a robot)(fleet-type "master")))))   
        ;  (format t "Master robotid: ~s ~%" (get-value master-robot 'name))
        ;  (send-job request (get-value master-robot 'name))))        
        )

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;      (show-robots)
;      (show-stations)
;      (show-locations)
;      (show-jobs)
;      (show-fleet)
;      (show-parking-locations)

      ;;send robots that still wait for the master the GO
      ;;this is not used anymore, since the master is defined by the master components only and they have allready been checked!
      (let ((init-robots (tcl-kb-query-all :key '(is-a state) :value `((is-a robot)(state ADDREQUEST)))))
        (dolist (robot init-robots)
        (format t "MASTER_ROBOT is ready send GO for the others waiting robots~%")
          (send-job '(ADDOK)(get-value robot 'name))))
      ;(let ((init-robots (tcl-kb-query-all :key '(is-a state) :value `((is-a robot)(state ADDREQUEST))))
      ;      (master-robot (tcl-kb-query :key '(is-a fleet-type) :value '((is-a robot)(fleet-type "master")))))
      ;       (cond 
      ;         ( (and (not (equal master-robot nil)) (not (equal (get-value master-robot 'state) 'INIT)) (not (equal init-robots nil)))
      ;           (format t "MASTER_ROBOT is ready send GO for the others waiting robots~%")
      ;           (dolist (robot init-robots)
      ;             (send-job '(ADDOK)(get-value robot 'name))))
      ;         ( (not (equal init-robots nil))
      ;           (format t "There are robots at the add list, the master is not ready, yet.~%"))))
                 
	 
           ;(manual-robots (tcl-kb-query-all :key '(is-a mode state performs-task) :value `((is-a robot)(mode MANUAL)(state IDLE)(performs-task nil))))
      (let ((manual-jobs (tcl-kb-query-all :key '(is-a life-cycle-state manual-assigned) :value `((is-a job)(life-cycle-state NOTSTARTED)(manual-assigned T)))))
             (handle-manual-assigned-jobs manual-jobs))

      (let ((automatic-robots (tcl-kb-query-all :key '(is-a mode state performs-task) :value `((is-a robot)(mode AUTO)(state IDLE)(performs-task nil))))
            (automatic-jobs (tcl-kb-query-all :key '(is-a life-cycle-state manual-assigned) :value `((is-a job)(life-cycle-state NOTSTARTED)(manual-assigned nil)))))
             (handle-automatic-jobs automatic-jobs automatic-robots))

      (let ((automatic-robots (tcl-kb-query-all :key '(is-a mode state performs-task is-parked) :value `((is-a robot)(mode AUTO)(state IDLE)(performs-task nil)(is-parked nil))))
            (manual-robots    (tcl-kb-query-all :key '(is-a mode state performs-task is-parked) :value `((is-a robot)(mode MANUAL)(state IDLE)(performs-task nil)(is-parked nil)))))
             (handle-idle-robots manual-robots automatic-robots))

      (let ((all-robots (tcl-kb-query-all :key '(is-a) :value `((is-a robot)))))
        (format t "List of all robots: ")
        (dolist (robot all-robots)
          (format t "~a " (get-value robot 'name)))
        (format t "~%"))

 ;  (time   (process-qos-job-parameters))

  (format t "LOOP DONE~%")))) 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun process-qos-job-parameters ()
    ;(send-remaining-job-duration)
)





;method for calls with nil
(defclass JOB-DISPACTHER-MAIN-CLASS ()
  ((memory :accessor kb-memory :initform nil)))


(defun handle-manual-assigned-jobs (manual-jobs)
  (format t "There are ~s jobs MANUAL,NOTSTARTED.~%" (list-length manual-jobs))

  (dolist (job manual-jobs)
    (format t "Try to assign job ~s to robot ~s " (get-value job 'id) (get-value job 'robotid))
    
  ;;TODO this could be done with the std lisp interface module stuff
           ;; (PushJob TYPE JOBID priority robotid start-location start-belt end-location end-belt)
           ;; PushJob RobotCommissioning JOBID PRIORITY ROBOTINOID COMMISSIONINGROBOT FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
  (cond
    ((equal 'IDLE (get-value (tcl-kb-query :key '(is-a name) :value `((is-a robot)(name ,(get-value job 'robotid)))) 'state))
      (cond 

       ;; PushJob TestJob JOBID PRIORITY ROBOTINOID
       ((equal (get-value job 'type) 'TestJob)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid)) (get-value job 'robotid)))

        ((equal (get-value job 'type) 'DeliverFromTo)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'start-station) ,(get-value job 'start-belt) ,(get-value job 'end-station) ,(get-value job 'end-belt)) (get-value job 'robotid)))
        ((equal (get-value job 'type) 'GotoPosition)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'goal-pose)) (get-value job 'robotid)))

        ;; PushJob PalletizeItem JOBID PRIORITY ROBOTINOID STARTSTATION palletize-STATION DROP-OFF-STATION 
        ((equal (get-value job 'type) 'PalletizeItem)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'start-station)  ,(get-value job 'palletize-station) ,(get-value job 'drop-off-station) ) (get-value job 'robotid)))

        ;; PushJob DeliverPallet JOBID PRIORITY ROBOTINOID STARTSTATION palletize-STATION DROP-OFF-STATION 
        ((equal (get-value job 'type) 'DeliverPallet)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'palletize-station) ,(get-value job 'drop-off-station) ) (get-value job 'robotid)))


        ;; collaborative job
        ((equal (get-value job 'type) 'RobotCommissioning)



          ;; PushJob RobotCommissioning JOBID PRIORITY Manipulation ROBOTINOID COMMISSIONINGROBOT [ORDER_ITEM QUANTITY]+
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Manipulation ,(get-value job 'robotid) ,(get-value job 'commissioning-robot) ,(get-value job 'commission-order)) (get-value job 'commissioning-robot))
          ;; PushJob RobotCommissioning JOBID PRIORITY Transportation ROBOTINOID COMMISSIONINGROBOT FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Transportation ,(get-value job 'robotid) ,(get-value job 'commissioning-robot) ,(get-value job 'start-location) ,(get-value job 'start-belt) ,(get-value job 'end-location) ,(get-value job 'end-belt) ,(get-value job 'commission-order)) (get-value job 'robotid)))


 

	;; ProductProduction job
        ((equal (get-value job 'type) 'ProductProduction)
          ;; PushJob ProductProduction JOBID PRIORITY Transportation ROBOTINOID manipulation-robot FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Transportation ,(get-value job 'robotid) ,(get-value job 'manipulation-robot) ,(get-value job 'start-location) ,(get-value job 'start-belt) ,(get-value job 'end-location) ,(get-value job 'end-belt) ,(get-value job 'productiondata)) (get-value job 'robotid))
          ;; PushJob ProductProduction JOBID PRIORITY Manipulation ROBOTINOID manipulation-robot FROMSTATION FROMBELT TOSTATION TOBELT [ORDER_ITEM QUANTITY]+
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Manipulation ,(get-value job 'robotid) ,(get-value job 'manipulation-robot) ,(get-value job 'start-location) ,(get-value job 'start-belt) ,(get-value job 'end-location) ,(get-value job 'end-belt) ,(get-value job 'productiondata)) (get-value job 'manipulation-robot)))
        


        ((equal (get-value job 'type) 'MPSDocking)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'docking-action) ,(get-value job 'docking-station-id) ,(get-value job 'docking-to-belt)) (get-value job 'robotid)))
        ((equal (get-value job 'type) 'FollowPerson)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid)) (get-value job 'robotid)))
        ((equal (get-value job 'type) 'MPSLoading)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'loading-action) ,(get-value job 'stationid)) (get-value job 'robotid)))
        ((equal (get-value job 'type) 'BatteryChargerDocking)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'docking-action)) (get-value job 'robotid)))
        ((equal (get-value job 'type) 'RobotGripper)
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,(get-value job 'robotid) ,(get-value job 'gripper-action)) (get-value job 'robotid)))
        (T 
          (format t "ERROR unkown job type!~%")))


       (tcl-kb-update :key '(is-a id) :value `((is-a job) (id ,(get-value job 'id)) (life-cycle-state RUNNING)))
       (tcl-kb-update :key '(is-a name) :value `((is-a robot) (name ,(get-value job 'robotid)) (performs-task ,(get-value job 'id)) (state BUSY) ))
       (format t " --> job assigned~%"))
    (T
      (format t " --> Could not assign job, robot busy~%")))))



(defun handle-automatic-jobs (automatic-jobs automatic-robots)
  (format t "There are ~s jobs AUTO,NOTSTARTED and ~s AUTO robots free.~%" (list-length automatic-jobs)(list-length automatic-robots))
  (cond
    ((and (< 0 (list-length automatic-jobs)) (< 0 (list-length automatic-robots)))
      (format t "DO SOMETHING.~%")
      (let ((pddl-fact nil)
           (pddl-domain nil)
           (pddl-result nil))
        (setf pddl-fact (export-pddl-fact-jobdispatch 0))
        (setf pddl-domain (export-pddl-domain-jobdispatch))
        (cond
          ((setf pddl-result(tcl-query :server 'symbolicplanner :service 'query :request (list
                '(planner metric-ff)
                `(domain ,(format nil "[~a]" (get-output-stream-string pddl-domain)))
                `(fact     ,(format nil "[~a]" (get-output-stream-string pddl-fact))))))
            ; everything ok
            (cond
              ((import-pddl-jobdispatch pddl-result)
                ; everything ok
                (format t "imported planner result - OK~%"))
              (T
                ; could not import plan
                (format t "imported planner result -no plan available~%"))))

          (T
            (format t  "ERROR query symbolic planner: ~s~%" pddl-result)))))))


;policy is to park any IDLE robot
(defun handle-idle-robots (manual-robots automatic-robots)
  (let ((free-parking-locations (tcl-kb-query-all :key '(is-a type parking-state) :value '((is-a location)(type parking)(parking-state free)))))
  (format t "There are ~s MANUAL and ~s AUTO idle not parked robots, ~s FREE parking locations.~%" (list-length manual-robots)(list-length automatic-robots)(list-length free-parking-locations))
  (cond
    ((and (< 0 (list-length free-parking-locations)) (or (< 0 (list-length automatic-robots)) (< 0 (list-length manual-robots))))
      (format t "--> Park idle robots.~%")
      ;start with AUTO robots
      (let ((pddl-fact nil)
           (pddl-domain nil)
           (pddl-result nil))
        (setf pddl-fact (export-pddl-fact-park-robots automatic-robots free-parking-locations))
        (setf pddl-domain (export-pddl-domain-park-robot))
        (cond
          ((setf pddl-result(tcl-query :server 'symbolicplanner :service 'query :request (list
                '(planner metric-ff)
                `(domain ,(format nil "[~a]" (get-output-stream-string pddl-domain)))
                `(fact     ,(format nil "[~a]" (get-output-stream-string pddl-fact))))))
            ; everything ok
            (cond
              ((import-pddl-park-robot pddl-result)
                ; everything ok
                (format t "imported planner result - OK~%"))
              (T
                ; could not import plan
                (format t "imported planner result -no plan available~%"))))

          (T
            (format t  "ERROR query symbolic planner: ~s~%" pddl-result))))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;TEST
;        (tcl-kb-update 
;         :key '(is-a name) 
;         :value `(
;            (is-a robot)
;            (name 99)
;            (state IDLE)
;            (mode AUTO)
;            (performs-task nil)
;            (current-symbolic-position 1)))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun send-command-to-all-robots (request)
  (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
         (dolist (robot robot-list)
           (format t " Forward command to robot: ~s~%" (get-value robot 'name))
           (send-job request (get-value robot 'name)))))


(defun send-job-state-change-to-fleet-com (job)
  (cond 
    ((< (get-value job 'id) 0)
      (format t "Internal job do not push update~%"))
    (T
      (cond 
        ((equal (get-value job 'type) 'DeliverFromTo)
          (tcl-send :server 'highlevelcommand :service 'result :param 
                (format nil "JobInfo robotinoid:~d jobid:~d priority:~d fromstation:~d frombelt:~d tostation:~d tobelt:~d state:~d life-cycle-state:~d" 
                (get-value job 'robotid) (get-value job 'id) 
                (get-value job 'priority) 
                (get-value job 'start-station) (get-value job 'start-belt) 
                (get-value job 'end-station) (get-value job 'end-belt) 
                (get-value job 'state) (get-value job 'life-cycle-state) )))
       ((equal (get-value job 'type) 'MPSDocking)
          (tcl-send :server 'highlevelcommand :service 'result :param 
                (format nil "JobInfo robotinoid:~d jobid:~d stationid:~d tobelt:~d state:~d life-cycle-state:~d" 
                (get-value job 'robotid) (get-value job 'id) (get-value job 'docking-station-id) (get-value job 'docking-to-belt) 
                (get-value job 'state) (get-value job 'life-cycle-state) )))

      (T 
        ;GotoPosition
        ;MPSLoading
        ;BatteryChargerDocking
        ;FollowPerson
        ;RobotGripper
        ;ParkRobot
        ;RobotCommissioning

        (tcl-send :server 'highlevelcommand :service 'result :param 
                (format nil "JobInfo robotinoid:~d jobid:~d state:~d life-cycle-state:~d" 
                (get-value job 'robotid) (get-value job 'id) 
                (get-value job 'state) (get-value job 'life-cycle-state) )))))))


(defun send-job-error-change-to-fleet-com (job)
  (cond
   ((equal (get-value job 'id) -1)
     (format t "send-job-error-change-to-fleet-com: ~s"(get-value job 'error-state))
     (tcl-send :server 'highlevelcommand :service 'result :param (format nil "CommandInfo robotinoid:~d state:~s" (get-value job 'robotid) (concatenate 'string "Error-" (get-value job 'error-state)))))
   (T
     (tcl-send :server 'highlevelcommand :service 'result :param 
                (format nil "JobError robotinoid:~d jobid:~d error:~s" 
                (get-value job 'robotid) (get-value job 'id) 
                (get-value job 'error-state) )))))


(defun send-command-info-to-fleet-com (request)
  (tcl-send :server 'highlevelcommand :service 'result :param (format nil "CommandInfo robotinoid:~d state:~s" (second request)(third request))))

(defun save-positions-from-kb-to-file ()
  (cond
    ((not (equal (probe-file (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default")) nil))
      (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/positions.lisp") :direction :output :if-exists :supersede :if-does-not-exist :create)
                 (if str
                  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a location)))))   
                    (dolist (obj obj-list)
                      (format str "~s~%" (get-kb-entry-serialize obj)))))))
    (T
      (format t "Error MAP Default link is not set --> NOT ABLE TO SAVE POSITIONS~%"))))

(defun save-stations-from-kb-to-file ()
  (cond
    ((not (equal (probe-file (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default")) nil))
      (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/stations.lisp") :direction :output :if-exists :supersede :if-does-not-exist :create)
                 (if str
                  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a station)))))   
                    (dolist (obj obj-list)
                      (format str "~s~%" (get-kb-entry-serialize obj)))))))
    (T
      (format t "Error MAP Default link is not set --> NOT ABLE TO SAVE STATIONS~%"))))


(defun load-positions-from-file-to-KB ()
 (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/positions.lisp") :if-does-not-exist nil)
   (if str
     (loop for entry = (read str nil)
       while entry do (tcl-kb-add-entry entry)))))

(defun load-stations-from-file-to-KB ()
 (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/stations.lisp") :if-does-not-exist nil)
   (if str
     (loop for entry = (read str nil)
       while entry do (tcl-kb-add-entry entry)))))

