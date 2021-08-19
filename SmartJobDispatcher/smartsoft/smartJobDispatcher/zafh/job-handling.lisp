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
;  Author: Matthias Lutz, Matthias Rollenhagen, Timo Blender
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

(defun handle-job (request)
  "This dunction handles the Job requests"

  (format t "[handleJob] found job~%")
  (let ((job-action (cdr (assoc 'action request)))
        (job-id (cdr (assoc 'job-id request))))
    (format t "[handleJob] job action: ~s~%" job-action)
    (format t "[handleJob] job id: ~s~%" job-id)

    (cond

      ((string-equal job-action "push-job")
        (let* ((push-job (cdr (assoc 'job request)))
               (push-job-type (string-downcase (string (cdr (assoc 'type push-job)))))
               (push-job-priority (cdr (assoc 'priority request)))
               (push-job-robotid (cdr (assoc 'robot-id request)))
              )
	  (format t "[handleJob] push-job type: ~s robot-id: ~s ~%" push-job-type push-job-robotid)
          (cond

          ;; goto-position - OK
          ((string-equal push-job-type "GotoPosition")
            ;(multiple-value-bind (position-id) (values (read-from-string (cdr (assoc 'position-id push-job))))
            (multiple-value-bind (position-id) (values (cdr (assoc 'position-id push-job)))
              (format t "Goto Position: ~s ~%" position-id)
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,job-id) 
                                   (state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,push-job-priority)
                                   ;;[Timo]
                                   ;(robots (,(if (equal push-job-robotid -1) nil `(,push-job-robotid))))
                                   (robotid ,(if (equal push-job-robotid -1) nil push-job-robotid))
				    (manual-assigned ,(if (equal push-job-robotid -1) nil T))
                                   (goal-pose ,position-id)
                                   (type ,(read-from-string push-job-type))
                                   (raw-msg ,request)))))
          
          ((string-equal push-job-type "RobotCommissioning")
              (multiple-value-bind (commissioningrobotid from-box-station-id from-box-belt to-station-id to-belt order-items job-part)
              (values (cdr (assoc 'commissioning-robot-id push-job)) (cdr (assoc 'from-box-station-id push-job)) (cdr (assoc 'from-box-station-belt push-job)) (cdr (assoc 'to-station-id push-job)) (cdr (assoc 'to-station-belt push-job)) (cdr (assoc 'order-items push-job)) (read-from-string (string-downcase (string (cdr (assoc 'job-part push-job))))))
                                   
              (format t "Timo: RobotCommissioning from Robot: ~s to Station ~s Belt ~s ~%" commissioningrobotid to-station-id to-belt)
              (tcl-kb-update :key '(is-a id) :value `(
                                   (is-a job) 
                                   (id ,job-id)
                                   ;;[Timo]
                                   ;(type ,push-job-type)
                                   (type ,(read-from-string push-job-type))
                                   (state NOTSTARTED)
                                   (error-state NOERROR) 
                                   (priority ,push-job-priority)
                                   ;;[Timo]
                                   ;(robots (,(if (equal push-job-robotid -1) nil `(,push-job-robotid))))
                                   (robotid ,(if (equal push-job-robotid -1) nil push-job-robotid))
                                   (manual-assigned ,(if (equal push-job-robotid -1) nil T))
                                   (commissioning-robot ,commissioningrobotid)
                                   (start-location ,from-box-station-id) (start-belt ,from-box-belt)
                                   (end-location ,to-station-id) (end-belt ,to-belt)
                                   (raw-msg ,request)
                                   ;;[Timo]
                                   (commission-order ,order-items)
                                   
                                   ))))
                                   ;;TODO NEEDS TO BE IMPLEMENTED 
                                   ;;(commission-order ,((lambda () (setf tmp nil) (loop for (a b) on (nthcdr 10 request) by #'cddr do (push (list a b) tmp)) tmp)))))))


            (T
              (format t "[handleJob] Error not supported PUSHJOB!~%")))))

      (T
        (format t "[handleJob] Error not supported JOB action!~%")))))
