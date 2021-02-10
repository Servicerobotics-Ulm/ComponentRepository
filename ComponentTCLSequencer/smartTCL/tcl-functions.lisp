;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2010 Andreas Steck
;;                      2014 Matthias Lutz
;;
;;		lutz@hs-ulm.de
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;      University of Applied Sciences
;;      Prittwitzstr. 10
;;      89075 Ulm
;;      Germany
;;
;;
;;  This library is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU Lesser General Public
;;  License as published by the Free Software Foundation; either
;;  version 2.1 of the License, or (at your option) any later version.
;;
;;  This library is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;;  Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public
;;  License along with this library; if not, write to the Free Software
;;  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;;
;;--------------------------------------------------------------------------

;;tcl-format
;format with timestamp as prefix and fixed destination (t) stdout
;example:  (tcl-format "Data: ~s ~a" "Hallo" 1)
(defun tcl-format (control-string &rest arguments)
  (apply #'format t (concatenate 'string "[" (multiple-value-bind 
                                             (second minute hour date month year)
                                             (get-decoded-time)
                                             (format NIL "~d-~d-~2,'0d ~2,'0d:~2,'0d:~2,'0d" year month date hour minute second))
                                         "] " 
                                         control-string) 
                    arguments))


;; tcl-event-message
(defun tcl-event-message ()
  *MSG*)


;; tcl-current-tcb-name
(defun tcl-current-tcb-name ()
  (tcb-name *CURRENT-INSTANCE*))


;; tcl-kb-add-entry
(defun tcl-kb-add-entry (values)
  (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-add-entry ',values)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component

;; tcl-kb-update
(defun tcl-kb-update (&key key value)
  (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-update :key ',key :value ',value)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component
;;  (update-kb *MEMORY* key value))

;; tcl-kb-update-batch
(defun tcl-kb-update-batch (&key updates-list (delete-key nil delete-key-supplied-p) (delete-value nil))
  (cond 
   (delete-key-supplied-p 
     (communication *SMARTSOFT* (list 'special  *KB-MODULE-NAME* 'query "kbQuery" `(kb-update-batch :updates-list ',updates-list  :delete-key ',delete-key :delete-value ',delete-value))))
  (T 
    (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* 'query "kbQuery" `(kb-update-batch :updates-list ',updates-list )))))
  (smartsoft-result *SMARTSOFT*))

;; tcl-kb-delete
(defun tcl-kb-delete (&key key value)
  (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-delete :key ',key :value ',value)))
  (smartsoft-result *SMARTSOFT*))
;;changed kb to external component
;;  (delete-kb *MEMORY* key value))


;; tcl-kb-query
(defun tcl-kb-query (&key key value)
  (let ((result nil))
  (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-query :key ',key :value ',value)))
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
       (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-query-all :key ',key :value ',value)))
       (setf resString (smartsoft-result *SMARTSOFT*))
       (cond 
         ((equal nil resString)
           nil) 
       (T
         (dolist (res resString)
           (setf result (append result (list (make-instance 'kb-entry :data res)))))
         result))))
;;changed kb to external component
;  (query-kb-all *MEMORY* key value))


;; tcl-kb-register-chained-entry
(defun tcl-kb-register-chained-entry (&key key value)
  (communication *SMARTSOFT* (list 'special *KB-MODULE-NAME* *KB-MODULE-NAME* 'query "kbQuery" `(kb-register-chained-entry :key ',key :value ',value)))
  (smartsoft-result *SMARTSOFT*))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; tcl-read-sync-variable
(defun tcl-read-sync-variable (&key name)
  (read-sync-variable *CURRENT-INSTANCE* name))

(defun tcl-reset-sync-variable (&key name)
  (reset-sync-variable *CURRENT-INSTANCE* name))

(defun tcl-write-sync-variable (&key name value)
  (write-sync-variable *CURRENT-INSTANCE* name value))

(defun tcl-wait-for-sync-variable (&key event-name variable-name handler)
  (wait-for-sync-variable *CURRENT-INSTANCE* event-name variable-name handler))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; tcl-bind-var
(defun tcl-bind-var (&key name value)
  (bind-var *CURRENT-INSTANCE* name (apply-substitution value (tcb-env-vars *CURRENT-INSTANCE*)))
  '(SUCCESS))


;; tcl-unbind-var
(defun tcl-unbind-var (&key name)
  (remove-var-from-slot *CURRENT-INSTANCE* 'env-vars name)
  '(SUCCESS))


;; tcl-clear-env-vars
(defun tcl-clear-env-vars ()
  (clear-vars *CURRENT-INSTANCE* 'env-vars)
  '(SUCCESS))


;; tcl-abort
(defun tcl-abort ()
  (abort-tcb-all *CURRENT-INSTANCE*)
  (tcl-delete-events)
  '(SUCCESS))


;; tcl-abort-child-tcbs-only
(defun tcl-abort-child-tcbs-only ()
  "Abort only the child blocks wihtout deleting the own events!"
  (abort-tcb-all *CURRENT-INSTANCE*)
  '(SUCCESS))



;; tcl-delete-plan
(defun tcl-delete-plan ()
  (abort-tcb-all *CURRENT-INSTANCE*)
  (setf (tcb-stack *CURRENT-INSTANCE*) nil)
  '(SUCCESS))


;; tcl-push
(defun tcl-push (&key tcb)
  (push-tcb *CURRENT-INSTANCE* tcb)
  '(SUCCESS))

;; tcl-push-back
(defun tcl-push-back (&key tcb)
  (push-back-tcb *CURRENT-INSTANCE* tcb)
  '(SUCCESS))

;; tcl-push-plan
(defun tcl-push-plan (&key plan)
  (format t "push elements to plan: ~s ~%" plan)
  (dolist (step (reverse plan))
;    (format t "push-tcl: ~s" step)
    (push-tcb *CURRENT-INSTANCE* step))
    ;;(push-plan *CURRENT-INSTANCE* :tcb step))
  '(SUCCESS))

;; tcl-push-back-plan
(defun tcl-push-back-plan (&key plan)
  (dolist (step plan)
    (push-back-tcb *CURRENT-INSTANCE* step))
    ;;(push-plan *CURRENT-INSTANCE* :tcb step))
  '(SUCCESS))


;; tcl-pop
(defun tcl-pop ()
  (pop-tcb *CURRENT-INSTANCE*)
  '(SUCCESS))

;; tcl-activate-event
(defun tcl-activate-event (&key name handler server service mode param)
  (activate-event *CURRENT-INSTANCE* name handler server service mode param)
  '(SUCCESS))

;; tcl-activate-event
(defun tcl-activate-event-direct (&key name handler server service mode param module module-inst)
  (activate-event-direct *CURRENT-INSTANCE* name handler server service mode param module module-inst)
  '(SUCCESS))


 ;(tcl-activate-event :handler 'handler-timer-expired :service 'relative :mode 'single :param '(1 100))



;; tcl-activate-timer-event
(defun tcl-activate-timer-event (&key name handler service mode param)
  (activate-event *CURRENT-INSTANCE* name handler *TIMER-MODULE-NAME* service mode param)
  '(SUCCESS))

;; tcl-delete-events
(defun tcl-delete-events ()
  (loop
    (let ((evt (pop (tcb-events *CURRENT-INSTANCE*))))
      (if (null evt) (return) (destroy-event *CURRENT-INSTANCE* (cdr evt))) ;call destory only if event exists
      (if (null (tcb-events *CURRENT-INSTANCE*)) (return))))
  '(SUCCESS()))
    


;; tcl-delete-event
(defun tcl-delete-current-event ()
  (destroy-event *CURRENT-INSTANCE* *EVT*)
  '(SUCCESS))

;; tcl-delete-event
(defun tcl-delete-event (&key name)
  (let ((event (member name (tcb-events *CURRENT-INSTANCE*) :key #'car)))
    (if event
      (progn (format t "tcl-delete-event - Event to destroy input: ~a --> name: ~a object: ~a~%" name (car (first event)) (cdr (first event)))
             (destroy-event *CURRENT-INSTANCE* (cdr (first event))))
      (format t "tcl-delete-event - Event to destroy input: ~a --> No Event found to destroy!~%" name)))
  '(SUCCESS))

;; tcl-send
(defun tcl-send (&key server service param)
  (let ( (module (tcb-module *CURRENT-INSTANCE*))
  (module-inst (tcb-module-inst *CURRENT-INSTANCE*)))
  (communication *SMARTSOFT* (list module module-inst server 'command service (apply-substitution param (tcb-env-vars *CURRENT-INSTANCE*))))
  '(SUCCESS)))

(defun tcl-send-direct (&key server service param module module-inst)
"This function is using the send outside of the context of a task block"
  (if *DEDBUG-CI* (format t "Server ~s~%" Server))
  (if *DEDBUG-CI* (format t "Module ~s~%" module))
  (if *DEDBUG-CI* (format t "ModuleInst ~s~%" module-inst))
  (communication *SMARTSOFT* (list module module-inst server 'command service (apply-substitution param (tcb-env-vars *CURRENT-INSTANCE*))))
  '(SUCCESS))


;; tcl-query
(defun tcl-query (&key server service request)
  (let ( (module (tcb-module *CURRENT-INSTANCE*))
       (module-inst (tcb-module-inst *CURRENT-INSTANCE*)))
    (communication *SMARTSOFT* (list module module-inst server 'query service (apply-substitution request (tcb-env-vars *CURRENT-INSTANCE*))))
    (smartsoft-result *SMARTSOFT*)))

(defun tcl-query-direct (&key server service request module module-inst)
"This function is using the query outside of the context of a task block"
  (if *DEDBUG-CI* (format t "Server ~s~%" Server))
  (if *DEDBUG-CI* (format t "Module ~s~%" module))
  (if *DEDBUG-CI* (format t "ModuleInst ~s~%" module-inst))
  (communication *SMARTSOFT* (list module module-inst server 'query service (apply-substitution request (tcb-env-vars *CURRENT-INSTANCE*))))
    (smartsoft-result *SMARTSOFT*))


;; tcl-param --> send
;;add paramvalue ad workaround for xtext (SmartSoftV2)
(defun tcl-param (&key server slot value (paramvalue nil paramvalue-supplied-p))
  (if paramvalue-supplied-p
    (setf value paramvalue))
  (let ( (tmp (apply-substitution value (tcb-env-vars *CURRENT-INSTANCE*)))
         ;;fetch the module and the module-inst information for the current executing block!
         ;;needs to be done here since later on the context (tcb) is not known any more
         (module (tcb-module *CURRENT-INSTANCE*))
         (module-inst (tcb-module-inst *CURRENT-INSTANCE*))
         (comp (query-kb *MEMORY* '(is-a name) `((is-a component)(name ,server)))))
    (if *DEDBUG-CI* (format t "Slot ~s~%" slot))
    (if *DEDBUG-CI* (format t "Server ~s~%" Server))
    (if *DEDBUG-CI* (format t "Module ~s~%" module))
    (if *DEDBUG-CI* (format t "ModuleInst ~s~%" module-inst))
    ;; send param
    (communication *SMARTSOFT* (list module module-inst server 'param "param" (list slot tmp)))
    ;; set param in memory
    (cond
      ((not (null (find slot (get-value comp 'slots))))
        (update-kb *MEMORY* '(is-a name)
          `(
             (is-a component)
             (name ,server)
             (,slot ,tmp))))))
  '(SUCCESS))


(defun tcl-param-direct (&key server slot value module module-inst)
"This function is using the parameter outside of the context of a task block"
  (if *DEDBUG-CI* (format t "Slot ~s~%" slot))
  (if *DEDBUG-CI* (format t "Server ~s~%" Server))
  (if *DEDBUG-CI* (format t "Module ~s~%" module))
  (if *DEDBUG-CI* (format t "ModuleInst ~s~%" module-inst))
    ;; send param
  (communication *SMARTSOFT* (list module module-inst server 'param "param" (list slot value)))
  '(SUCCESS))


;; tcl-get-param --> read from memory
(defun tcl-get-param (&key server slot)
  (let ( (comp (query-kb *MEMORY* '(is-a name) `((is-a component)(name ,server)))))
    (get-value comp slot)))


;; tcl-state
(defun tcl-state (&key server state)
  (let ((tmp (apply-substitution state (tcb-env-vars *CURRENT-INSTANCE*)))
         ;;fetch the module and the module-inst information for the current executing block!
         ;;needs to be done here since later on the context (tcb) is not known any more
        (module-inst (tcb-module-inst *CURRENT-INSTANCE*))
        (module (tcb-module *CURRENT-INSTANCE*)))
    ;; send state
    (communication *SMARTSOFT* (list module module-inst server 'state "state" tmp))
    ;; set state in memory
    (update-kb *MEMORY* '(is-a name)
      `(
         (is-a component)
         (name ,server)
         (state ,tmp))))
  '(SUCCESS))

(defun tcl-state-direct (&key server state module module-inst)
"This function is using the state outside of the context of a task block"
  (if *DEDBUG-CI* (format t "Server ~s~%" Server))
  (if *DEDBUG-CI* (format t "Module ~s~%" module))
  (if *DEDBUG-CI* (format t "ModuleInst ~s~%" module-inst))
  (communication *SMARTSOFT* (list module module-inst server 'state "state" state))
    '(SUCCESS))


(define-condition component-life-cycle-error (error)
  ((text :initarg :text :reader text)))

; (tcl-life-cycle-wait-for-state :module "KBModule" :module-inst 'KBMODINST :server "kb"  :state "Neutral")

;; tcl-life-cycle-wait-for-state
(defun tcl-life-cycle-wait-for-state (&key module module-inst server state)
  (let ((tmp (apply-substitution state (tcb-env-vars *CURRENT-INSTANCE*))))
    (cond
      ((or (equal module *WIRING-MODULE-NAME*) (equal module *FETCHEVENT-MODULE-NAME*) (equal module *COMPONENT-MODULE-NAME*) (equal module *TIMER-MODULE-NAME*))
        '(SUCCESS))
      (T
        ;; send state
        (if (communication *SMARTSOFT* (list module module-inst server 'state "waitforlifecyclestate" tmp))
         '(SUCCESS)
         (error 'component-life-cycle-error
           :text "Component is not in the requested state and an Error occured!"))))))


; (tcl-get-state :module "KBModule" :module-inst 'KBMODINST :server "kb")

;; tcl-get-state
(defun tcl-get-state (&key module module-inst server)
  (communication *SMARTSOFT* (list module module-inst server 'state "getstate" nil))
  (smartsoft-result *SMARTSOFT*))


;;TODO check if this code is correct and the most recent version
;; tcl-wiring-connect
;; macro used by the xtext tcl grammar
;(defmacro tcl-ci-wiring-connect (&rest sections)
;  (let* ( (clientComp    (coord-inter-get-value ':clientComp     sections))
;          (wiringName    (coord-inter-get-value ':service        sections))
;          (serverComp    (coord-inter-get-value ':serverComp     sections))
;          (serverService (coord-inter-get-value ':serverService  sections)))
;    `(tcl-wiring-connect :clientComp ',clientComp :wiringName ',wiringName :serverComp ',serverComp :serverService ',serverService)))

;  (tcl-ci-wiring-connect :clientComp "1.SmartFileMover" :wiringName "FileReadQueryClient" :serverComp "master.SmartFileProvider" :serverService "ReadFileQueryServer")


;  tcl-module-wiring-connect 


;; tcl-wiring-connect
(defun tcl-wiring-connect (&key clientComp wiringName serverComp serverService)
  (let ((tmp (apply-substitution (list clientComp wiringName serverComp serverService) (tcb-env-vars *CURRENT-INSTANCE*)))
        (module (tcb-module *CURRENT-INSTANCE*))
        (module-inst (tcb-module-inst *CURRENT-INSTANCE*)))
    ;; wiring
    (communication *SMARTSOFT* (list 'special *WIRING-MODULE-NAME* (list module module-inst) nil 'connect tmp)))
  '(SUCCESS))


;;TODO check if this code is correct and the most recent version
;; tcl-wiring-disconnect
;; macro used by the xtext tcl grammar
;(defmacro tcl-ci-wiring-disconnect (&rest sections)
;  (let* ( (clientComp    (coord-inter-get-value ':clientComp     sections))
;          (wiringName    (coord-inter-get-value ':service        sections)))
;    `(tcl-wiring-disconnect :clientComp ',clientComp :wiringName ',wiringName :serverComp ',serverComp :serverService ',serverService)))



;; tcl-wiring-disconnect
(defun tcl-wiring-disconnect (&key clientComp wiringName)
  (let ((tmp (apply-substitution (list clientComp wiringName) (tcb-env-vars *CURRENT-INSTANCE*)))
        (module (tcb-module *CURRENT-INSTANCE*))
        (module-inst (tcb-module-inst *CURRENT-INSTANCE*)))
    ;; wiring
    (communication *SMARTSOFT* (list 'special *WIRING-MODULE-NAME* (list module module-inst) nil 'disconnect tmp)))
  '(SUCCESS))


;; tcl-adaptation
(defun tcl-adaptation(&key state)
  (cond
    ((equal state T)
      (format t "adaptation switchen on~%")
      (setf *ADAPTATION* T))
    ((equal state nil)
      (format t "adaptation switchen off~%")
      (setf *ADAPTATION* nil))
    (T
      (format t "adaptation unsupported state -- current state: ~:[off~;on~] ~%" *ADAPTATION*))))


;; tcl-tcb-avail
(defun tcl-tcb-avail (&key name priority in out precondition flag)
  (update-kb *MEMORY* '(is-a name in-vars out-vars precondition priority)
    `( (is-a tcb)
       (name ,name)
       (priority ,priority)
       (in-vars ,in)
       (out-vars ,out)
       (precondition ,precondition)
       (avail-flag ,flag))))


