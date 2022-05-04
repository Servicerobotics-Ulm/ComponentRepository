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
    (format t "Comp ~s~%" comp)
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


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun show-fleet ()
  (let ((fleet-list (tcl-kb-query-all :key '(is-a) :value '((is-a fleet-class)))))
    (dolist (fleet fleet-list)
    (format t "~%--------------------------------~%")
    (format t "F L E E T ------------------------~%")
    (format t "  id                  : ~s ~%" (get-value fleet 'id))
    (format t "  state               : ~s ~%" (get-value fleet 'state))
    (format t "  robots              : ~s ~%" (get-value fleet 'robots))
    (format t "  robot-mapping       : ~s ~%" (get-value fleet 'robot-mapping)))))

(defun show-jobs ()
  (let ((job-list (tcl-kb-query-all :key '(is-a) :value '((is-a job)))))
    (dolist (job job-list)
    (format t "~%--------------------------------~%")
    (format t "J O B --------------------------~%")
    (format t "  id                 : ~s ~%" (get-value job 'id))
    (format t "  type               : ~s ~%" (get-value job 'type))
    (format t "  robotid            : ~s ~%" (get-value job 'robotid))
    (format t "  manual-assigned    : ~s ~%" (get-value job 'manual-assigned))
    (format t "  push-time          : ~s ~%" (get-value job 'push-time))
    (format t "  start-time         : ~s ~%" (get-value job 'start-time))
    (format t "  end-time           : ~s ~%" (get-value job 'end-time))
    (format t "  lc-state           : ~s ~%" (get-value job 'life-cycle-state))
    (format t "  error-state        : ~s ~%" (get-value job 'error-state))
    (format t "  state              : ~s ~%" (get-value job 'state)))))

(defun show-robots ()
  (let ((robot-list (tcl-kb-query-all :key '(is-a) :value '((is-a robot)))))
    (dolist (robot robot-list)
    (format t "~%--------------------------------~%")
    (format t "R O B O T ------------------------~%")
    (format t "  name              : ~s ~%" (get-value robot 'name))
    (format t "  ip                : ~s ~%" (get-value robot 'robotip))
    (format t "  fleet-type        : ~s ~%" (get-value robot 'fleet-type))
    (format t "  base-component    : ~s ~%" (get-value robot 'base-component))
    (format t "  laser-component   : ~s ~%" (get-value robot 'laser-component))
    (format t "  path-nav-component: ~s ~%" (get-value robot 'path-nav-component))
    (format t "  performs-task     : ~s ~%" (get-value robot 'performs-task))
    (format t "  state             : ~s ~%" (get-value robot 'state))
    (format t "  sub-state         : ~s ~%" (get-value robot 'sub-state))
    (format t "  mode              : ~s ~%" (get-value robot 'mode))
    (format t "  is-job-on-time    : ~s ~%" (get-value robot 'is-job-on-time))
    (format t "  is-docked         : ~s ~%" (get-value robot 'is-docked))
    (format t "  box-loaded        : ~s ~%" (get-value robot 'box-loaded))
    (format t "  is-parked         : ~s ~%" (get-value robot 'is-parked ))
    (format t "  pos         : ~s ~%" (get-value robot 'pos ))
    )))

(defun show-locations ()
  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a location)))))
    (dolist (obj obj-list)
      (format t "~%--------------------------------~%")
      (format t "L O C A T I O N ------------------~%")
      (format t "name                       : ~s ~%" (get-value obj 'name))
      (format t "  type                     : ~s ~%" (get-value obj 'type))
      (format t "  approach-type            : ~s ~%" (get-value obj 'approach-type))
      (format t "  approach-region-pose     : ~s ~%" (get-value obj 'approach-region-pose))
      (format t "  approach-region-dist     : ~s ~%" (get-value obj 'approach-region-dist))
      (format t "  approach-exact-pose      : ~s ~%" (get-value obj 'approach-exact-pose))
      (format t "  approach-exact-dist      : ~s ~%" (get-value obj 'approach-exact-dist))
      (format t "  approach-exact-safetycl  : ~s ~%" (get-value obj 'approach-exact-safetycl))
      (format t "  orientation-region       : ~s ~%" (get-value obj 'orientation-region))
      (format t "  orientation-exact        : ~s ~%" (get-value obj 'orientation-exact))
      (format t "  parking-state            : ~s ~%" (get-value obj 'parking-state )))))

(defun show-parking-locations ()
  (let ((obj-list     (tcl-kb-query-all :key '(is-a type) :value '((is-a location)(type parking)))))
    (dolist (obj obj-list)
      (format t "~%--------------------------------~%")
      (format t "name                       : ~s ~%" (get-value obj 'name))
      (format t "  type                     : ~s ~%" (get-value obj 'type))
      (format t "  parking-state            : ~s ~%" (get-value obj 'parking-state ))))
  (let ((obj-list     (tcl-kb-query-all :key '(is-a type) :value '((is-a location)(type charging)))))
    (dolist (obj obj-list)
      (format t "~%--------------------------------~%")
      (format t "name                       : ~s ~%" (get-value obj 'name))
      (format t "  type                     : ~s ~%" (get-value obj 'type))
      (format t "  parking-state            : ~s ~%" (get-value obj 'parking-state )))))

(defun show-stations ()
  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a station)))))
    (dolist (obj obj-list)
      (format t "~%--------------------------------~%")
      (format t "S T A T I O N --------------------~%")
      (format t "id                         : ~s ~%" (get-value obj 'id))
      (format t "  approach-location        : ~s ~%" (get-value obj 'approach-location))
      (format t "  belt-count               : ~s ~%" (get-value obj 'belt-count))
      (format t "  type                     : ~s ~%" (get-value obj 'type))
      (format t "  docking-type             : ~s ~%" (get-value obj 'docking-type)))))

(defun show-station-types ()
  (let ((obj-list     (tcl-kb-query-all :key '(is-a) :value '((is-a station-type)))))
    (dolist (obj obj-list)
      (format t "~%--------------------------------~%")
      (format t "S T A T I O N - T Y P E S --------------------~%")
      (format t "name                         : ~s ~%" (get-value obj 'name))
      (format t "  min-station-width          : ~s ~%" (get-value obj 'min-station-width))
      (format t "  max-station-width          : ~s ~%" (get-value obj 'max-station-width))
      (format t "  offs-bet-center-o-belts    : ~s ~%" (get-value obj 'offset-between-center-of-belts))
      (format t "  offs-left-ref-to-1Belt-cent: ~s ~%" (get-value obj 'offset-left-reflector-to-1stBelt-center))
      (format t "  ir-dock-stop-dist-l2-min   : ~s ~%" (get-value obj 'ir-dock-stop-dist-l2-min))
      (format t "  ir-dock-stop-dist-l2-max   : ~s ~%" (get-value obj 'ir-dock-stop-dist-l2-max))
      (format t "  ir-dock-stop-dist-l1       : ~s ~%" (get-value obj 'ir-dock-stop-dist-l1))
      (format t "  ir-dock-center-sensor-zero : ~s ~%" (get-value obj 'ir-dock-center-sensor-zero))
      (format t "  docking-type               : ~s~%"  (get-value obj 'docking-type))
      (format t "  commincation-type          : ~s~%"  (get-value obj 'commincation-type))
      (format t "  number-of-belts            : ~s~%"  (get-value obj 'number-of-belts))
      (format t "  max-reflector-dist         : ~s~%"  (get-value obj 'max-reflector-dist))
      (format t "  laser-dock-stop-dist       : ~s~%"  (get-value obj 'laser-dock-stop-dist))
      (format t "  description                : ~s ~%" (get-value obj 'description)))))

(defun show-job-timing ()
  (let ((job-list     (tcl-kb-query-all :key '(is-a) :value '((is-a job-timing)))))
    (dolist (job job-list)
      (format t "~%--------------------------------~%")
      (format t "J O B --------------------~%")
      (format t "  type                     : ~s ~%" (get-value job 'type))
      (format t "  robotid                  : ~s ~%" (get-value job 'robotid))
      (format t "  timing                   : ~s ~%" (get-value job 'timing)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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

