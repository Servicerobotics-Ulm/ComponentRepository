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

; (send-remaining-job-duration)

(defun send-remaining-job-duration ()
  (let* ((duration (calculate-remaining-job-duration)))
    (cond
      ((not (null duration))
        (cl-elasticsearch:add-to-index 1 `((|type| . "remaining-job-duration") (|duration| .  ,duration)) "_doc" "remaining_job_duration"))
      (T
        (format t "Error calculating remaining job duration!~%")))))

(defun average (list) (/ (reduce #'+ list) (length list)))



;;TODO FIXME this is done only for the master 
(defun calculate-remaining-job-duration ()
  (let ((open-jobs (tcl-kb-query-all :key '(is-a life-cycle-state) :value `((is-a job)(life-cycle-state NOTSTARTED))))
        (job-list  (tcl-kb-query-all :key '(is-a robotid) :value '((is-a job-timing)(robotid 6))))
        (robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot))))
        (job-avg-list nil)
        (result 0))
    
    (dolist (job job-list)
      (let ((tmp nil))
      ;(format t "Type: ~a~%" (get-value job 'type))
      ;(format t "timing: ~a~%" (get-value job 'timing))
      (dolist (timing (get-value job 'timing))
        (if (and (not (null (first timing))) (not (null (second timing))))
          (push  (- (second timing) (first timing)) tmp)))

      ;(format t "durations: ~a~%" tmp)
      (if (null tmp)
        (push `( ,(get-value job 'type) . 0) job-avg-list)
        (push `( ,(get-value job 'type) . ,(average (remove 0 tmp))) job-avg-list))))


    (format t "calc-rem-job-dur list: ~a~%" job-avg-list)
    
    (dolist (job open-jobs)
      (let ((tmp nil))
        (setf tmp (assoc (get-value job 'type) job-avg-list))
        (cond
          ((not (null tmp))
            ;(format t "found match: ~s~%" tmp)
            (setf result (+ result (cdr tmp)))))))

    (format t "calc-rem-job-dur full duration: ~a" result)
    ;(format t "(list-length robot-list): ~a~%" (list-length robot-list))
    (setf result (/ result (list-length robot-list)))
    (format t " RESULT: ~a~%" result)
    result))
    
  

(defun send-all-jobs (jobs)
  (dolist (job jobs)
    (send-job-to-elastic job)))


(defun send-job-to-elastic (job)
  (let ((formated-start-time nil)
        (formated-end-time nil))

    (cond 
      ((or (null (get-value job 'start-time)) (null (get-value job 'end-time)))
        (setf formated-end-time "1900-01-01T00:00:01")
        (setf formated-start-time "1900-01-01T00:00:01"))
      (T
        (multiple-value-bind (second minute hour day month year) (decode-universal-time (get-value job 'start-time))
          (setf formated-start-time (format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second)))
        (multiple-value-bind (second minute hour day month year) (decode-universal-time (get-value job 'end-time))
          (setf formated-end-time (format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second)))))

    (cl-elasticsearch:add-to-index (get-value job 'id) 
      `((|job-id| . ,(get-value job 'id))
        (|job-type| . ,(get-value job 'type))
        (|robot-id| . ,(get-value job 'robotid))
        (|life-cycle-state| . ,(if (null (get-value job 'life-cycle-state)) (if (equal (get-value job 'error-state) 'NOERROR) (format nil "NOTSTARTED") (format nil "ERROR")) (get-value job 'life-cycle-state)))
        (|start_time| . ,formated-start-time)
        (|end_time| . ,formated-end-time)
        (|type| . "fleet-job-info"))
       "_doc" "fleet-jobs")))
        






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; OLD - NOT USED!?
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun send-job-timings (timings robotid)
  (format t "send-job-timings:~a~%" timings)
  (dolist (obj (second timings))
    (multiple-value-bind (second minute hour day month year) (decode-universal-time (first obj))
      (multiple-value-bind (e_second e_minute e_hour e_day e_month e_year) (decode-universal-time (second obj))
      (format t "ID: ~a Time: " robotid) (format t "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D   " year month day hour minute second)
      (format t "End- Time: ") (format t "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D~%" e_year e_month e_day e_hour e_minute e_second)
    
      (cl-elasticsearch:add-to-index-noid `((|robot| . ,(format nil "~a" robotid))
                                            (|start_time| .  ,(format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second))
                                            (|end_time| . ,(format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" e_year e_month e_day e_hour e_minute e_second)) 
                                            (|job-type| . ,(format nil "~a" (first timings)))
                                            (|type| . "job-timing-info"))
                                            "_doc" "robot_fleet")))))



;      (if (null (get-value job 'start-time))
;        (setf formated-start-time "1900-01-01T00:00:01")
;        (multiple-value-bind (second minute hour day month year) (decode-universal-time (get-value job 'start-time))
;          (setf formated-start-time (format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second))))
;      (if (null (get-value job 'end-time))
;        (setf formated-end-time "1900-01-01T00:00:01")
;        (multiple-value-bind (second minute hour day month year) (decode-universal-time (get-value job 'end-time))
;          (setf formated-end-time (format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second))))


;    
;      (multiple-value-bind (e_second e_minute e_hour e_day e_month e_year) (decode-universal-time (second obj))
;      (format t "ID: ~a Time: " robotid) (format t "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D   " year month day hour minute second)
;      (format t "End- Time: ") (format t "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D~%" e_year e_month e_day e_hour e_minute e_second)
;    
;      (cl-elasticsearch:add-to-index-noid `((|robot| . ,(format nil "~a" robotid))
;                                            (|start_time| .  ,(format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" year month day hour minute second))
;                                            (|end_time| . ,(format nil "~4,'0D-~2,'0D-~2,'0DT~2,'0D:~2,'0D:~2,'0D" e_year e_month e_day e_hour e_minute e_second)) 
;                                            (|job-type| . ,(format nil "~a" (first timings)))
;                                            (|type| . "job-timing-info"))
;                                            "_doc" "robot_fleet")))))


;    (cl-elasticsearch:add-to-index "14" '((robot . "1") (start_time . (first obj))
;    (end_time . "2018-07-05T13:30:00") (job-type . "GoTo")
;    (type . "job-timing-info")) "_doc" "matthias")))



 ; (cl-elasticsearch:add-to-index "14" '((robot . "1") (start_time . "2018-07-05T12:30:00")
 ;  (end_time . "2018-07-05T13:30:00") (job-type . "GoTo")
 ;  (type . "job-timing-info")) "_doc" "matthias"))
