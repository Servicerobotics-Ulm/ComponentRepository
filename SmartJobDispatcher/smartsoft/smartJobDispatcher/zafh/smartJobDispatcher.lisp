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
	;(load (format nil "/opt/smartsoft/src/components/SmartJobDispatcher/smartJobDispatcher/zafh/split-sequence.lisp"))
	;(load (format nil "/opt/smartsoft/src/components/SmartJobDispatcher/smartJobDispatcher/split-sequence.fasl"))
	;(load (format nil "/opt/smartsoft/src/components/SmartJobDispatcher/smartJobDispatcher/zafh/decode-msg.lisp"))
 	;(load (format nil "/home/shaik/SOFTWARE/smartsoft/repos/ComponentRepository/SmartJobDispatcher/smartsoft/smartJobDispatcher/zafh/command-handling.lisp"))
 	;(load (format nil "/home/shaik/SOFTWARE/smartsoft/repos/ComponentRepository/SmartJobDispatcher/smartsoft/smartJobDispatcher/zafh/job-handling.lisp"))
	(load "~/SOFTWARE/smartsoft/repos/ComponentRepository/SmartJobDispatcher/smartsoft/smartJobDispatcher/zafh/command-handling.lisp")
	(load "~/SOFTWARE/smartsoft/repos/ComponentRepository/SmartJobDispatcher/smartsoft/smartJobDispatcher/zafh/job-handling.lisp")  	
)

;; why undefined function decode-msg?
;(load (format nil "/opt/smartsoft/src/components/SmartJobDispatcher/smartJobDispatcher/zafh/decode-msg.lisp"))

;(load (format nil "/opt/smartsoft/src/components/SmartJobDispatcher/smartJobDispatcher/split-sequence.fasl"))
(define-modify-macro appendf (&rest args) 
   append "Append onto list")

(defun runJobDispachter ()
"This function contains the main loop of the component!"
 (loop 
  (let ((request nil)
        (tmp nil)
        (flag 0))
    (format t "[Timo] ZAFH JobDispatcher ~%")
    (setf tmp (wait-for-message))
    (format t "[Timo] Received Message (FleetCom or Robots): ~s~%" tmp)
    (format t "[Timo] First Char: ~s~%" (char tmp 0))
    
    (cond 
    	((string-equal "(" (char tmp 0))
    	  (format t "[Timo] is json or not jobinfo~%")
    	  (setf request (decode-msg tmp))
    	)	
    	(t
    	  (format t "[Timo] is jobinfo~%")
    	      
    	      (setf request (read-from-string tmp))	
    	  	
    	      (cond	
		     ((typep request '(simple-array *))
		      (format t "[Timo] Condition matched!")
		      (let ((tmp nil)
			   (outlist nil))
			(setf tmp (SPLIT-SEQUENCE:split-sequence #\space request))  
			(dolist (x tmp)
			   (setf outlist (append outlist (last (SPLIT-SEQUENCE:split-sequence #\: x)))))
		      (setf request (read-from-string (format nil "~a" outlist))))))
		
		
		(cond 
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
          )))
	      
	      (setf flag 1)
    	  
    	)
    )
    
    ;;parse msg to an alist
    ;; 15-01-21 Timo
    ;(setf request (decode-msg (subseq tmp 1 (- (length tmp) 1))))
    
    
    (format t "[Timo] Parsed Result: ~s~%" request)
    
    
    
    (format t "=========================>>> HANDLER HIGHLEVEL  FLEET COMMAND: ~s ~%~%" request)
    
    ;;TODO CHECK IF THIS IS STILL NEEDED
    ;;convert messages to forward to fleetcom to a lisp list to parse them
    ;;this madness is because of the key:value definition in the fleet com!


    (format t "request: ~s~%" request)
    (format t "[Timo] flag: ~s~%" flag)
	
    (cond 
    	((= flag 0)
    ;; seperate different msg types   
    (let ((msg-type (cdr (assoc 'msg-type request))))
      (cond
         ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
         ;; TEST CODE new interface !
         ;; Genertate function call from JSON message
	#|
	Example JSON Message:

	{ "msg-type" : "create",   
	  "obj-type" : "rack",   
	  "obj-list" : [
		           {   "id" :  "shelf123", 
		               "type" : "inclined-shelf", 
		               ...
		           } ]  }
	|#

         ((or (string-equal msg-type "create") (string-equal msg-type "read") (string-equal msg-type "update") (string-equal msg-type "delete") )
            (let* ((obj-type (cdr (assoc 'obj-type request)))
                   ;(handle-function-name (symbol-function (find-symbol (string-upcase (concatenate 'string "handle-" msg-type "-" obj-type) )))))
                   (handle-function-name (symbol-function (find-symbol (string-upcase (concatenate 'string "handle-" msg-type) )))))
              
              (funcall handle-function-name `(,(read-from-string obj-type) ,(cdr (assoc 'obj-list request))))))


         ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



         ((string-equal msg-type "command")
          ;; COMMAND HANDLING
          (format t "[Timo] handle-command ~%")
          (handle-command request))

        ((string-equal msg-type "query")
          ;; QUERY HANDLING
          (handle-query request))
  
        ((string-equal msg-type "job")
          ;; JOB HANDLING
          (format t "[Timo] handle-job ~%") 
          (handle-job request))

        ((string-equal msg-type "robot-msg")
          ;; ROBOT MSG HANDLING 
          (handle-robot-msg request))

        ((string-equal msg-type "state-change-msg")
          ;; STATE CHANGE HANDLING 
          (handle-state-change request))

        ((string-equal msg-type "debug-msg")
          ;; DEBUG HANDLING 
          (format t "[handleDebug] found command~%")
          (let* ((msg (cdr (assoc 'msg request)))
                 (msg-type (cdr (assoc 'type msg))))

            (format t "[handleDebug] found msg: ~s~%" msg)
            (format t "[handleDebug] msg type: ~s~%" msg-type)

            (cond
              ;; DEBUG AND SPECIAL MSGS

              ; ON_COMPONENT_SHUTDOWN
              ((string-equal msg-type "on-component-shutdown")
                ;; FLEET --> INIT
                (tcl-kb-update :key '(is-a id) :value '((is-a fleet-class)(id 0)(state SHUTDOWN)))
                (format t "ON_COMPONENT_SHUTDOWN exit loop~%")
                (return-from runJobDispachter 'quit))

              ((string-equal msg-type "exit")
                (return))

              ((string-equal msg-type "quit")
                (quit))
     
              (T
                (format t "Error not supported COMMAND!~%")))))

        (T 
          (format t "[Timo] Unkown MSG TYPE (JSON): ~a~%" msg-type))))
      )
      )


      (show-robots)
;      (show-stations)
;      (show-locations)
      (show-jobs)
      (show-fleet)
      (show-parking-locations)

      ;;send robots that still wait for the master the GO
      ;;this is not used anymore, since the master is defined by the master components only and they have allready been checked!
      ;(let ((init-robots (tcl-kb-query-all :key '(is-a state) :value `((is-a robot)(state ADDREQUEST)))))
      ;  (dolist (robot init-robots)
      ;  (format t "MASTER is ready send GO to the waiting robots~%")
      ;    (send-job (encode-msg `((msg-type . "robot-msg") (msg . ((type . "add-robot-request-response") (robot-id . ,(get-value robot 'name)) (value . "add-ok"))))) (get-value robot 'name))))
          
      (let ((init-robots (tcl-kb-query-all :key '(is-a state) :value `((is-a robot)(state ADDREQUEST)))))
        (dolist (robot init-robots)
        	(format t "MASTER_ROBOT is ready send GO for the others waiting robots~%")
          	(send-job '(ADDOK)(get-value robot 'name))
        )
      )
      
      
      
      ;(let ((init-robots (tcl-kb-query-all :key '(is-a state) :value `((is-a robot)(state ADDREQUEST))))
      ;      (master-robot (tcl-kb-query :key '(is-a fleet-type) :value '((is-a robot)(fleet-type "master")))))
      ;       (cond 
      ;         ( (and (not (equal master-robot nil)) (not (equal (get-value master-robot 'state) 'INIT)) (not (equal init-robots nil)))
      ;           (format t "MASTER_ROBOT is ready send GO for the others waiting robots~%")
      ;           (dolist (robot init-robots)
      ;             (send-job '(ADDOK)(get-value robot 'name))))
      ;         ( (not (equal init-robots nil))
      ;           (format t "There are robots at the add list, the master is not ready, yet.~%"))))
                 

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
      ;; handle manual jobs
           ;(manual-robots (tcl-kb-query-all :key '(is-a mode state performs-task) :value `((is-a robot)(mode MANUAL)(state IDLE)(performs-task nil))))
      (let ((manual-jobs (tcl-kb-query-all :key '(is-a state manual-assigned) :value `((is-a job)(state NOTSTARTED)(manual-assigned T)))))
             (handle-manual-assigned-jobs manual-jobs))

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
      ;; handle manual jobs
      (let ((automatic-robots (tcl-kb-query-all :key '(is-a mode state performs-task) :value `((is-a robot)(mode AUTO)(state IDLE)(performs-task nil))))
            (automatic-jobs (tcl-kb-query-all :key '(is-a state manual-assigned) :value `((is-a job)(state NOTSTARTED)(manual-assigned nil)))))
             (handle-automatic-jobs automatic-jobs automatic-robots))

      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
      ;; handle/park idle robots 
      (let ((automatic-robots (tcl-kb-query-all :key '(is-a mode state performs-task is-parked) :value `((is-a robot)(mode AUTO)(state IDLE)(performs-task nil)(is-parked nil))))
            (manual-robots    (tcl-kb-query-all :key '(is-a mode state performs-task is-parked) :value `((is-a robot)(mode MANUAL)(state IDLE)(performs-task nil)(is-parked nil)))))
             (handle-idle-robots manual-robots automatic-robots))

  (format t "LOOP DONE~%"))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;method for calls with nil
(defclass JOB-DISPACTHER-MAIN-CLASS ()
  ((memory :accessor kb-memory :initform nil)))

(defun tcl-query (&key server service request)
  (communication *SMARTSOFT* (list server 'query service request ))
  (smartsoft-result *SMARTSOFT*))

;; tcl-param --> send
(defun tcl-param (&key server slot value)
  (let ( 
         (full-slot (apply-parameter-substitution server slot))
         (comp (query-kb *MEMORY* '(is-a name) `((is-a component)(name ,server)))))
    (format t "Slot ~s~%" slot)
    (format t "Server ~s~%" Server)
    (format t "result ~s~%" (apply-parameter-substitution server slot))
    ;; send param
    (communication *SMARTSOFT* (list server 'command 'command (list full-slot value))))
  '(SUCCESS))

;; tcl-send
(defun tcl-send (&key server service param)
  (communication *SMARTSOFT* (list server 'command service param ))
  '(SUCCESS))

;; tcl-state
(defun tcl-state (&key server state)
    ;; send state
    (communication *SMARTSOFT* (list server 'state 'state state))
  '(SUCCESS))

(define-condition component-life-cycle-error (error)
  ((text :initarg :text :reader text)))

;; tcl-life-cycle-wait-for-state
(defun tcl-life-cycle-wait-for-state (&key server state)
    ;; send state
    (if (communication *SMARTSOFT* (list server 'state 'lifecycle-wait state))
       '(SUCCESS)
       (error 'component-life-cycle-error
         :text "Component is not in the requested state and an Error occured!")))

;; tcl-kb-add-entry
(defun tcl-kb-add-entry (values)
  (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-add-entry ',values)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component

;; tcl-kb-update
(defun tcl-kb-update (&key key value)
  (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-update :key ',key :value ',value)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component
;;  (update-kb *MEMORY* key value))

;; tcl-kb-update-batch
(defun tcl-kb-update-batch (&key updates-list (delete-key nil delete-key-supplied-p) (delete-value nil))
  (cond 
   (delete-key-supplied-p 
     (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-update-batch :updates-list ',updates-list  :delete-key ',delete-key :delete-value ',delete-value))))
  (T 
    (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-update-batch :updates-list ',updates-list )))))
  (smartsoft-result *SMARTSOFT*))

;; tcl-kb-delete
(defun tcl-kb-delete (&key key value)
  (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-delete :key ',key :value ',value)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component
;;  (delete-kb *MEMORY* key value))


;; tcl-kb-query
(defun tcl-kb-query (&key key value)
  (let ((result nil))
  (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-query :key ',key :value ',value)))
  (setf result (smartsoft-result *SMARTSOFT*))
  (cond 
    ((equal nil result)
      nil) 
    (T
      (make-instance 'kb-entry :data result)))))
;;changed kb to external component
;;  (query-kb *MEMORY* key value))

;; tcl-kb-query-all
(defun tcl-kb-query-all (&key key value)
  (let ((result nil)
      (resString nil))
       (communication *SMARTSOFT* (list 'knowledgebase 'query 'kb `(kb-query-all :key ',key :value ',value)))
       (setf resString (smartsoft-result *SMARTSOFT*))
       (cond 
         ((equal nil resString)
           nil) 
       (T
         (dolist (res resString)
           (setf result (append result (list (make-instance 'kb-entry :data res)))))
         result))))

;; tcl-wiring-connect
(defun tcl-wiring-connect (&key clientComp wiringName serverComp serverService)
  (let ((tmp (list clientComp wiringName serverComp serverService)))
    ;; wiring
    (communication *SMARTSOFT* (list 'wiring 'wiring 'connect tmp)))
  '(SUCCESS))


;; tcl-wiring-disconnect
(defun tcl-wiring-disconnect (&key clientComp wiringName)
  (let ((tmp (list clientComp wiringName) ))
    ;; wiring
    (communication *SMARTSOFT* (list 'wiring 'wiring 'disconnect tmp)))
  '(SUCCESS))


;;TODO improve error handling in case get-robot-pose was not successful
(defun calculate-distance-to-start (robot job)
  (let* ((robotpose (get-robot-pose robot))
        (location-name (get-value job 'start-location))
        (locationpose (get-value (tcl-kb-query :key '(is-a name) :value `((is-a location)(name ,location-name))) 'approach-region-pose)))
    (format t "[calculate-distance-to-start] robotpose ~s locationpose ~s ~%" robotpose locationpose)
    (cond 
      ((and (not (equal nil robotpose)) (not (equal nil locationpose)))
        (let ((x1 (first robotpose))
             (y1 (second robotpose))
             (x2 (first locationpose))
             (y2 (second locationpose)))
         (sqrt (+ (expt (- x2 x1) 2) (expt (- y2 y1) 2) ))))
      (T
        nil))))
        
(defun get-robot-pose (robot)
   (let ((component-name (get-value robot 'base-component)))
    (tcl-wiring-connect :clientComp "master.SmartJobDispatcher" :wiringName "baseStateQueryClient"
                               :serverComp component-name :serverService "basestatequery"))
    (let ((robotpose (tcl-query :server 'base :service 'pose)))
      (tcl-wiring-disconnect :clientComp "master.SmartJobDispatcher" :wiringName "baseStateQueryClient")
      (cond 
       ((equal robotpose '(SMART DISCONNECTED))
        nil)
       (T
        robotpose))))

(defun handle-manual-assigned-jobs (manual-jobs)
  (format t "There are ~s jobs MANUAL,NOTSTARTED.~%" (list-length manual-jobs))

  (dolist (job manual-jobs)
    (format t "Try to assign job ~s to robot ~s ~%" (get-value job 'id) (get-value job 'robotid))
    
    (format t "TYPE: ~s~%" (get-value job 'type))
    
  (cond
    ((equal 'IDLE (get-value (tcl-kb-query :key '(is-a name) :value `((is-a robot)(name ,(get-value job 'robotid)))) 'state))
      (cond 

        ((equal (get-value job 'type) 'GotoPosition)
          
          (format t "job-id ~s ~%" (get-value job 'id))
          (format t "robotid ~s ~%" (get-value job 'robotid))
          (format t "goal-pose ~s ~%" (get-value job 'goal-pose))
          
          
          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) 
            ,(get-value job 'robotid) ,(get-value job 'goal-pose)) (get-value job 'robotid))
        )
        
        ((equal (get-value job 'type) 'RobotCommissioning)

          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Manipulation 
          ,(get-value job 'robotid) ,(get-value job 'commissioning-robot) ,(get-value job 'commission-order)) 
          (get-value job 'commissioning-robot))

          (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) Transportation 
          ,(get-value job 'robotid) ,(get-value job 'commissioning-robot) ,(get-value job 'start-location) 
          ,(get-value job 'start-belt) ,(get-value job 'end-location) ,(get-value job 'end-belt) 
          ,(get-value job 'commission-order)) (get-value job 'robotid))
        )
        
        (T 
          (format t "[Timo] Unkown job type!~%")
        )
      )

      (tcl-kb-update :key '(is-a id) :value `((is-a job) (id ,(get-value job 'id)) (life-cycle-state RUNNING)))
      (tcl-kb-update :key '(is-a name) :value `((is-a robot) (name ,(get-value job 'robotid)) (performs-task 
	,(get-value job 'id)) (state BUSY) ))
      (format t " --> job assigned~%"))
    (T
      (format t " --> Could not assign job, robot busy~%")
    )
  )
 )
)

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
;;TODO THIS EFFECTS THE OLD INTERFACE!!
;        ((equal (get-value job 'type) 'DeliverFromTo)
;          (tcl-send :server 'highlevelcommand :service 'result :param 
;                (format nil "JobInfo robotinoid:~d jobid:~d priority:~d fromstation:~d frombelt:~d tostation:~d tobelt:~d state:~d" 
;                (get-value job 'robotid) (get-value job 'id) 
;                (get-value job 'priority) 
;                (get-value job 'start-station) (get-value job 'start-belt) 
;                (get-value job 'end-station) (get-value job 'end-belt) 
;                (get-value job 'state) )))
;        ((equal (get-value job 'type) 'MPSDocking)
;          (tcl-send :server 'highlevelcommand :service 'result :param 
;                (format nil "JobInfo robotinoid:~d jobid:~d stationid:~d tobelt:~d  state:~d" (get-value job 'robotid) (get-value job 'id) (get-value job 'docking-station-id) (get-value job 'docking-to-belt) (get-value job 'state) )))

      (T 
        ;GotoPosition
        ;MPSLoading
        ;BatteryChargerDocking
        ;FollowPerson
        ;RobotGripper
        ;ParkRobot
        ;RobotCommissioning
;;        (tcl-send :server 'highlevelcommand :service 'result :param (format nil "JobInfo robotinoid:~d jobid:~d state:~d" (get-value job 'robotid) (get-value job 'id) (get-value job 'state) )))))))
          (tcl-send :server 'highlevelcommand :service 'result :param (encode-msg `((msg-type . "state-change-msg") (update . ((type . "job-state-change") (robot-id . ,(get-value job 'robotid)) (job-id . ,(get-value job 'id)) (state . ,(get-value job 'state)) (error-state . "NOERROR")))))))))))


(defun send-job-error-change-to-fleet-com (job)
  (cond
   ((equal (get-value job 'id) -1)
     (format t "send-job-error-change-to-fleet-com: ~s"(get-value job 'error-state))
;;     (tcl-send :server 'highlevelcommand :service 'result :param (format nil "CommandInfo robotinoid:~d state:~s" (get-value job 'robotid) (concatenate 'string "Error-" (get-value job 'error-state)))))
     (tcl-send :server 'highlevelcommand :service 'result :param (encode-msg `((msg-type . "state-change-msg") (update . ((type . "command-info") (robot-id . ,(get-value job 'robotid)) (state . ,(concatenate 'string "Error-" (get-value job 'error-state)))))))))

   (T
;;     (tcl-send :server 'highlevelcommand :service 'result :param (format nil "JobError robotinoid:~d jobid:~d error:~s" (get-value job 'robotid) (get-value job 'id) (get-value job 'error-state) )))))
     (tcl-send :server 'highlevelcommand :service 'result :param (encode-msg `((msg-type . "state-change-msg") (update . ((type . "job-state-change-error") (robot-id . ,(get-value job 'robotid)) (job-id . ,(get-value job 'id)) (error-state . ,(get-value job 'error-state)) (state . "ERROR")))))))))




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
       
       
       
       

;; ####################################################### decode-msg #######################################################
       
(defun decode-msg (input-msg)
  "This function takes a string in json or s-expr format and decodes it to a alist"
  (let ((parsed-msg nil)
        (string-to-parse nil)
        (tmp))
   ;(format t "Decode msg raw: ~s~%" input-msg)
   (if (wrapped-string-p input-msg)
     (setf string-to-parse (enroll-wrapped-string input-msg))
     (setf string-to-parse input-msg))
     
     (setf tmp string-to-parse)
   ;(format t "Decode msg: ~s~%" string-to-parse)
   ;(format t "Typeof msg: ~a~%" (type-of string-to-parse))
   
    ;;[Timo] Remove () from JSON message
    (setf string-to-parse (subseq string-to-parse 1 (- (length string-to-parse) 1)))

    (handler-case
      (with-input-from-string (s string-to-parse)
        (let ((cl-json:*json-symbols-package* nil))
          (setf parsed-msg (cl-json:decode-json s))
        )
      )
          
      (t (c)
        (format t "[decode-json-msg] WARNING decode msg json invaild try s-expr: ~a~%" c)
        (format t "[Timo] input-msg ~s~%" input-msg)
        (format t "[Timo] string-to-parse ~s~%" string-to-parse)
        (format t "[Timo] tmp ~s~%" tmp)
        
        ;[Timo]
        ;(setf parsed-msg (decode-sexpr string-to-parse))
        (setf parsed-msg (decode-sexpr tmp))
        
        (values nil c)))

    
    ;(format t "Parsed result ~s~%" parsed-msg)
    parsed-msg))

;;[Timo] Handles messages from robots in sexpr format
(defun decode-sexpr (input-msg)
  "This function takes a string in s-expr format and decodes it to a alist"
  (format t "[Timo] before read-from-string ~%")
  (let (
  	;(request (read-from-string (concatenate 'string "(" input-msg ")")))
  	(request (read-from-string input-msg))
        (parsed-msg nil)
       )
    (format t "[Timo]: sexpr ~s~%" request)
    (cond
    
      ((equal (first request) 'SystemState)
        (format t "[Timo] Message of SytemState:~s ~%" request) 
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
            (is-job-on-time ,(nth 12 request)) ))
      )
	
      ((equal (first request) 'AddRequest)
      	(format t "[Timo] AddRequest from robot: ~s type: ~s ~%" (second request) (third request))
        (let ((robots-in-fleet (get-value (tcl-kb-query :key '(is-a id) :value '((is-a job)(id 0))) 'robots)))
	     (tcl-kb-update :key '(is-a id) :value `( (is-a fleet-class) (id 0) 
	     (robots ,(push (second request) robots-in-fleet))))
        )
			(tcl-kb-update 
	 			:key '(is-a name) 
	 			:value `(
							(is-a robot)
							(name ,(second request))
							(state ADDREQUEST)
							(fleet-type ,(third request))
						)
			)
      )
      
      ((equal (first request) 'AddRobot)
        (format t "Message AddRobot:~s TYPE:~s ~%" request (tenth request))
        (format t "[Timo] AddRobot.State ~s ~%" (fifth request))
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

            (tcl-param :server 'highlevelcommand :slot 'DOMAINROBOTFLEET.FLEETMANAGERPARAMETER.ADDROBOT 
            :value `( ,(second request) ,(first (third request)) ,(second (third request)) ,(third (third request)) 
                  ,(first (ninth request)) ,(second (ninth request)) ,(first (nth 10 request)) ,(second (nth 10 request))))
            (tcl-param :server 'robotinorpcbridge :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.ADDROBOT 
                                 :value `( ,(second request) ,(first (third request)) ,(second (third request)) 
                                 ,(third (third request)) ,(first (ninth request)) ,(second (ninth request)) 
                                 ,(first (nth 10 request)) ,(second (nth 10 request))))
    
        (send-job '(AddRobotCompleted)(second request))
      )
      
      ((equal (first request) 'GetManualAcknowledge)
        (format t "GOT GetManualAcknowledge --> FORWARD~%")
        (tcl-send :server 'highlevelcommand :service 'result :param 
        	(format nil "GetManualAcknowledge robotinoid:~d " (second request))
        )
      )      
      
      ((equal (first request) 'ReplacePositions)
        (format t "[Timo] ReplacePosition [non-json]~%")
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
        
        
        ((equal (first request) 'ReloadDefaultPaths)
        (format t "ReloadDefaultPaths~%")
        (format t "Path: ~s ~%" (second request))
              (tcl-send :server 'navPath
                        :service 'sendPath 
                        :param '?path
              )
        '(SUCCESS()))
      

      
      (T
	(format t "[Timo] No matching sexpr message~%")
      )
    )
   parsed-msg ;[Timo] Return?
  )
)


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



;; ####################################################### decode-msg #######################################################


