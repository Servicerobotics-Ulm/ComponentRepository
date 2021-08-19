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



;; E X P O R T
(defun export-pddl-fact-jobdispatch (envid)
(format t "export-pddl-fact-jobdispatch with envid: ~s ~%" envid)
(let ((str (make-string-output-stream)))
  (format str "(define (problem select-robot-1)~%")
  (format str "  (:domain select-robot)~%")

  (let* ((robots (tcl-kb-query-all :key '(is-a state mode) :value `((is-a robot)(state idle)(mode auto))))
        (all-jobs (tcl-kb-query-all :key '(is-a life-cycle-state manual-assigned) :value `((is-a job)(life-cycle-state NOTSTARTED)(manual-assigned nil))))
        (jobs (loop 
                for job in all-jobs
                for i upto (- (length robots) 1)
                collect job)))
  ;;;; objects 
  (format str "   (:objects~%")
  (format t "robots: ~s~%" robots)
  (dolist (robot robots)
    (let ((name (get-value robot 'name)))
      (format str "    ROBOT-~a - robot~%" name)))
  (dolist (job jobs)
    (let ((id (get-value job 'id)))
      (format str "    TASK-~a - task~%" id)))
  (format str "   )~%")

  ;;;; init
  (format str "   (:init~%")

  ;; total-cost
  (format str "    (= (total-cost) 0)~%")
  (format str "    (= (undocking-cost) 10)~%")

  ;; distances todo we need to get this from path planning
  (dolist (robot robots)
    (let ((name (get-value robot 'name)))
    (dolist (job jobs)
      (let ((id (get-value job 'id))
            (distance (calculate-distance-to-start robot job)))
      (cond 
        ((equal distance nil)
          (format str "    (= (distance ROBOT-~a TASK-~s) 99999999)~%" name id))
        (T
          (format str "    (= (distance ROBOT-~a TASK-~s) ~d)~%" name id distance)))))))

  ;; robota are docked
  (dolist (robot robots)
    (let ((docked (get-value robot 'is-docked))
          (name (get-value robot 'name)))
      (if (eql docked T) (format str "    is-docked ROBOT-~a~%" name))))

  (format str ")~%")

  ;; goal
  (format str "  (:goal~%")
  (format str "    (and ~%")
  (dolist (job jobs)
    (let ((id (get-value job 'id)))
      (format str "    (is-performed TASK-~s)~%" id)))
  (format str "   ))~%")
  (format str "    (:metric minimize (total-cost)))")
  str)))

;; E X P O R T
(defun export-pddl-domain-jobdispatch ()
(let ((str (make-string-output-stream)))
  (format str "(define (domain select-robot)
 (:requirements :adl :typing)
  (:types robot task)
 
  (:predicates
     (robot ?robot - robot)
     (task ?task - task)
     (is-docked ?robot - robot)
     (performs-task ?robot - robot)
     (is-performed ?task - task))
  (:functions 
     (distance ?robot ?task)
     (undocking-cost)
     (total-cost))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; actions

  (:action undock-robot
    :parameters (?robot - robot)
    :precondition (and 
                      (is-docked ?robot))
    :effect (and 
                (not (is-docked ?robot))
                (increase (total-cost) (undocking-cost))))

  (:action perform-task
    :parameters (?robot - robot ?task - task)
    :precondition (and
                     (not (is-docked ?robot))
                     (not (performs-task ?robot))
                     (not (is-performed ?task)))
    :effect (and (performs-task ?robot)
                 (is-performed ?task)
                 (increase (total-cost) (distance ?robot ?task)))))")  
  str))



(defun import-pddl-jobdispatch(result)
  (let* (
         (status (second (assoc 'result (first result))))
         (plan   (second (assoc 'plan (first result)))))
        ;;(format t "import-pddl-tabletop-1: correct plan available: ~s ~%" result)
        (format t "status: ~s ~%" status)
        (cond 
          ((equal status 'error)
                ;no plan generated
                (format t "import-pddl-tabletop-1: no plan generated~%")
                nil)
          ((equal status 'ok)
            ;correct plan available
            (format t "import-pddl-tabletop-1: correct plan available~%")
              ;; iterate over plan steps ----------------------------------------------------------------------
              (dolist (step plan)
                (format t "Plan Step: ~s~%" step)
                (cond

		  ;; PERFORM-TASK
                  ((equal (first step) 'PERFORM-TASK)
                   (format t "DUMMY TASK FROM PLANNER ROBOT:~s TASK:~s ~%"(string-trim "ROBOT-" (second step)) (third step))
                   (let* ((robotname nil))
                     (handler-case
                       (progn
                         (setf robotname (parse-integer (string-trim "ROBOT-" (second step)))))
                       (error (e) (format t "String as robot name! e:~s ~%" e)
                         (setf robotname (intern (string-trim "ROBOT-" (second step))))))
                   
                     (let ((job (tcl-kb-query :key '(is-a id) :value `((is-a job)(id ,(parse-integer (subseq (format nil "~s" (third step)) 5)))))))
                       (format t "FOUND JOB~%")

                       (cond 
                         ((equal (get-value job 'type) 'DeliverFromTo)
		               ;; (PushJob TYPE JOBID priority robotid start-location start-belt start-manual end-location end-belt end-manual)
                           (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,robotname ,(get-value job 'start-location) ,(get-value job 'start-belt) ,(get-value job 'start-manual) ,(get-value job 'end-location) ,(get-value job 'end-belt),(get-value job 'end-manual)) robotname))

                         ((equal (get-value job 'type) 'GotoPosition)
		               ;; (PushJob TYPE JOBID priority robotid goal-pose)
                           (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,robotname ,(get-value job 'goal-pose)) robotname))
                         ((equal (get-value job 'type) 'RobotCommissioning)
                           (send-job `(PushJob ,(get-value job 'type) ,(get-value job 'id) ,(get-value job 'priority) ,robotname ,(get-value job 'commissioning-robot) ,(get-value job 'start-location) ,(get-value job 'start-belt) ,(get-value job 'start-manual) ,(get-value job 'end-location) ,(get-value job 'end-belt) ,(get-value job 'end-manual) ,(get-value job 'commission-order)) robotname)))


                       (tcl-kb-update :key '(is-a id) :value `((is-a job) 
                                                    (id ,(get-value job 'id)) (life-cycle-state running) (robotid ,robotname)))
                       (tcl-kb-update :key '(is-a name) :value `((is-a robot) 
                                                    (name ,robotname) (performs-task ,(get-value job 'id)))))))))
              T)
              ;;-----------------------------------------------------------------------------------------------

          (T
            ;unknown format
            (format t "[import-pddl-jobdispatch]: unknown format~%")
            nil))))
