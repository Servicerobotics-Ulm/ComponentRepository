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
(defun export-pddl-fact-park-robots (all-robots parking-spots)
(let ((str (make-string-output-stream)))
  (format str "(define (problem park-robot-1)~%")
  (format str "  (:domain park-robot)~%")

  (let* ((robots (loop 
                for robot in all-robots
                for i upto (- (length parking-spots) 1)
                collect robot)))
  ;;;; objects 
  (format str "   (:objects~%")
  (format t "robots: ~s~%" robots)
  (dolist (robot robots)
    (let ((name (get-value robot 'name)))
      (format str "    ROBOT-~a - robot~%" name)))
  (dolist (spot parking-spots)
    (let ((name (get-value spot 'name)))
      (format str "    PARKING-SPOT-~a - spot~%" name)))
  (format str "   )~%")

  ;;;; init
  (format str "   (:init~%")

  ;; total-cost
  (format str "    (= (total-cost) 0)~%")

  ;; distances todo we need to get this from path planning
  (dolist (robot robots)
    (let ((robot-name (get-value robot 'name)))
    (dolist (spot parking-spots)
      (let ((spot-name (get-value spot 'name))
            (distance (calculate-distance-to-start robot spot)))
      (cond 
        ((equal distance nil)
          (format str "    (= (distance ROBOT-~a PARKING-SPOT-~s) 99999999)~%" robot-name spot-name))
        (T
          (format str "    (= (distance ROBOT-~a PARKING-SPOT-~s) ~d)~%" robot-name spot-name distance)))))))

  (format str ")~%")

  ;; goal
  (format str "  (:goal~%")
  (format str "    (and ~%")
  (dolist (robot robots)
    (let ((name (get-value robot 'name)))
      (format str "    (is-parked ROBOT-~s)~%" name)))
  (format str "   ))~%")
  (format str "    (:metric minimize (total-cost)))")
  str)))

;; E X P O R T
(defun export-pddl-domain-park-robot ()
(let ((str (make-string-output-stream)))
  (format str "(define (domain park-robot)
 (:requirements :adl :typing)
  (:types robot spot)
 
  (:predicates
     (robot ?robot - robot)
     (spot ?spot - spot)
     (is-taken ?spot - spot)
     (is-parked ?robot - robot))
  (:functions 
     (distance ?robot ?spot)
     (total-cost))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; actions

  (:action park-robot
    :parameters (?robot - robot ?spot - spot)
    :precondition (and
                     (not (is-parked ?robot))
                     (not (is-taken ?spot)))
    :effect (and (is-parked ?robot)
                 (is-taken ?spot)
                 (increase (total-cost) (distance ?robot ?spot)))))")  
  str))



(defun import-pddl-park-robot(result)
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

		  ;; park-robot
                  ((equal (first step) 'park-robot)
                   (format t "DUMMY TASK FROM PLANNER ROBOT:~s SPOT:~s ~%"(string-trim "ROBOT-" (second step)) (third step))
                   (let* ((robotname nil))
                     (handler-case
                       (progn
                         (setf robotname (parse-integer (string-trim "ROBOT-" (second step)))))
                       (error (e) (format t "String as robot name! e:~s~%" e)
                         (setf robotname (intern (string-trim "ROBOT-" (second step))))))
                   
                     (let ((spot (tcl-kb-query :key '(is-a name) :value `((is-a location)(name ,(parse-integer (subseq (format nil "~s" (third step)) 13)))))))
                       (format t "FOUND LOCATION~%")

	               ;; (PushJob TYPE JOBID priority robotid goal-pose)
                       (send-job `(PushJob ParkRobot 'PARKING priority ,robotname ,(get-value spot 'name)) robotname)
                       (tcl-kb-update :key '(is-a name) :value `((is-a location) (name ,(get-value spot 'name)) (parking-state (T ,robotname))))
                       (tcl-kb-update :key '(is-a name) :value `((is-a robot) (name ,robotname) (performs-task 'PARKING) (is-parked (T ,(get-value spot 'name))))))))))
              T)
              ;;-----------------------------------------------------------------------------------------------

          (T
            ;unknown format
            (format t "[import-pddl-park-robot]: unknown format~%")
            nil))))
