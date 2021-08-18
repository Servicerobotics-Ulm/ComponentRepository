;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2010 Andreas Steck
;;
;;		steck@hs-ulm.de
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;	University of Applied Sciences
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



;;;##################################################################################
;;;##################################################################################
;;; TCB execution class which provides several standardized methods
;;;

(defclass TCB-EXECUTION-CLASS ()
  ( (name             :accessor tcb-name             :initform nil)
    (avail-flag       :accessor tcb-avail-flag       :initform nil)
    (in-vars          :accessor tcb-in-vars          :initform nil)
    (out-vars         :accessor tcb-out-vars         :initform nil)
    (out-var-mapping  :accessor tcb-out-var-mapping  :initform nil)
    (env-vars         :accessor tcb-env-vars         :initform nil)
    (ext-run-time-id  :accessor tcb-ext-run-time-id  :initform nil)
    (stack            :accessor tcb-stack            :initform nil)
    (rules            :accessor tcb-rules            :initform nil)
    (sync-variables   :accessor tcb-sync-variables   :initform nil) ; sync vars accessable
    (reg-sync-vars    :accessor tcb-reg-sync-vars    :initform nil) ; sync vars registered for wait!
    (events           :accessor tcb-events           :initform nil)
    (event-handler    :accessor tcb-event-handler    :initform nil)
    (current          :accessor tcb-current          :initform nil)
    (current-mode     :accessor tcb-current-mode     :initform nil)
    (current-result   :accessor tcb-current-result   :initform nil)
    (abort-action     :accessor tcb-abort-action     :initform nil)
    (module-inst      :accessor tcb-module-inst      :initform nil)
    (module           :accessor tcb-module           :initform nil)))



;;; constructor <instance> <signature>
;;;    description : tries to match <signature> against tcb's in *MEMORY*. If an appropriate tcb matched the slots are filled. 
;;;                  Otherwise the tcb-name is set to nil to indicate that no tcb could be matched.
;;;    example     : ...
;;;    returns     : an instance of a TCB-EXECUTION-CLASS
(defmethod initialize-instance :after ((instance tcb-execution-class) &key signature parent-sync-variables)
  (format t "initialize-instance: ~s ~%" signature)

  ;(format t "parent-sync-variables: ~s ~%" parent-sync-variables)
  (setf (slot-value instance 'sync-variables) parent-sync-variables)

  (setf *CURRENT-INSTANCE* instance)
  (let ( (result '(ERROR (NO TCB MATCHED)))
         (tcb-selected nil)
         (tcb-name nil)
         (tcb-module-inst nil)
         (precondition-eval nil)
         (tcb-match-list nil)
         (tcb-list nil))
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;;;; QUERY FOR TCBs
    (if *ADAPTATION*
      (setf tcb-list (query-kb-all *MEMORY* '(is-a avail-flag) '((is-a tcb)(avail-flag T))))
      (setf tcb-list (query-kb-all *MEMORY* '(is-a) '((is-a tcb)))))
    ;;;;
    (setf (slot-value instance 'name) nil)
    ;; parse signature
    (if *DEDBUG-CI* (format t "TCB sig: ~a~%" signature))
    (multiple-value-bind (name module-inst in-vars out-vars ext-run-time-id)
      (parse-tcl-signature signature)
 
      ;(format t "Result parse sig: ~s ~s ~s ~s~%"  name in-vars out-vars ext-run-time-id)
      (setf (slot-value instance 'in-vars) in-vars)
      (setf (slot-value instance 'out-vars) out-vars)
      (setf (slot-value instance 'ext-run-time-id) ext-run-time-id)

      ;; store out-variables in mapping slot -- default mapping is 'todo --> appropriate mapping will be updated after 
      ;; correct tcb matched
      (dolist (ov out-vars)
        (push (cons ov 'todo) (slot-value instance 'out-var-mapping)))

      ;;TODO CHECK THIS!!!
      (setf tcb-module-inst module-inst)
      (setf (slot-value instance 'module-inst) module-inst)
      (if *DEDBUG-CI* (format t "tcb-module-inst: ~a~%" tcb-module-inst))
      (setf tcb-name name))
    
    ;; try to match tcb and store them in tcb-match-list     
    ;;(dolist (tcb tcb-list)
    (dolist (tcb (sort tcb-list #'(lambda (tcb-1 tcb-2) (> (get-value tcb-1 'priority) (get-value tcb-2 'priority)))))
      (cond 
        ((not (null tcb))
          (setf (tcb-env-vars instance) (unify
                                          (append (list (get-value tcb 'name)) (get-value tcb 'in-vars) (list (length (get-value tcb 'out-vars))))
                                          (append (list tcb-name) (slot-value instance 'in-vars) (list (length (slot-value instance 'out-vars))))))
          (cond
            ((and (not (null (get-value tcb 'precondition))) (not (equal (tcb-env-vars instance) 'fail)))
	          (setf precondition-eval (eval (apply-substitution  (get-value tcb 'precondition) (tcb-env-vars instance)))))
              ;(format t "after evaluation precondition: ~s = ~s ~%" (get-value tcb 'precondition) precondition-eval))
            (T
              (setf precondition-eval T)))
          
          (cond
            ; tcb does not match
            ((or (equal (tcb-env-vars instance) 'fail) (null precondition-eval))
              nil)
               
            ; tcb does match
            (T
              (cond
                ((null tcb-match-list)
                  (push (list tcb (length (tcb-env-vars instance)) (tcb-env-vars instance)) tcb-match-list))
                
                ((= (length (tcb-env-vars instance)) (second (first tcb-match-list)))
                  (push (list tcb (length (tcb-env-vars instance)) (tcb-env-vars instance)) tcb-match-list))
                
                ((< (length (tcb-env-vars instance)) (second (first tcb-match-list)))
                  (setf tcb-match-list nil)
                  (push (list tcb (length (tcb-env-vars instance)) (tcb-env-vars instance)) tcb-match-list))
                
                (T
                  nil)))))))
    
    ;; with the co concept multiple matching (signature) tcbs could be in the list
    (cond
      ((not (null tcb-match-list))
        (if *DEDBUG-CI* (format t "tcb-match-list: ~s ~%" tcb-match-list))

        ; only select those which is in the matching module type (select by the instance name)
        (let* ((module-type (get-value (query-kb *MEMORY* '(is-a inst-name) `((is-a coordination-module) (inst-name ,tcb-module-inst))) 'type))
               (tcb-filtered-list (remove-if #'(lambda (x) (not (equal (get-value (first x) 'module) module-type))) tcb-match-list)))

               (if *DEDBUG-CI* (format t "tcb-filtered-list (module matched): ~s ~%" tcb-filtered-list))

               (cond 
                 ((not (null tcb-filtered-list))
                   ;; now just take the first one, the order should still be sorted by priority from above
                   (setf tcb-selected (first (first (reverse tcb-filtered-list)))) ;; just take first one
                   (setf (slot-value instance 'env-vars) (third (first tcb-filtered-list))))

                 (T
                   ; tcb not matching tcb found in the instance
                   (format t "ERROR no tcb found in module:~a module-ints:~a~%" module-type tcb-module-inst)
                   ; fallback use the matching tcb even if not in the correct module
                   (setf tcb-selected (first (first (reverse tcb-match-list)))) ;; just take first one
                   (setf (slot-value instance 'env-vars) (third (first tcb-match-list)))))))

      (T
        ;; be sure that tcb is set to nil whether no tcb matched
        (setf tcb-selected nil)))
    
    ;; store infos
    (cond
      ((not (null tcb-selected))
        ;(format t "tcb from library: ~s  -- MATCHED -- tcb instance name: ~s ~%"
        ;  (append (list (get-value tcb-selected 'name)) (get-value tcb-selected 'in-vars) (list (length (get-value tcb-selected 'out-vars))))
        ;  (append (list tcb-name) (slot-value instance 'in-vars) (list (length (slot-value instance 'out-vars)))))
        ;; fill slots plan - action
        (push-plan instance (get-value tcb-selected 'plan))
        ;; fill abort-action
        (setf (slot-value instance 'abort-action) (get-value tcb-selected 'abort-action))
        ;; fill signature (name in out)
        (setf (slot-value instance 'name) tcb-name)
        (setf (slot-value instance 'in-vars) (get-value tcb-selected 'in-vars))
        (setf (slot-value instance 'out-vars) (get-value tcb-selected 'out-vars))
        (setf (slot-value instance 'module) (get-value tcb-selected 'module))
	;; TODO CHECK THIS
        ;(setf (slot-value instance 'module-inst) (get-value tcb-selected 'module-inst))
        ;; fill rules
        (setf (slot-value instance 'rules) (get-value tcb-selected 'rules))
        ;; update out-variable mapping
        (let ((tcb-out-vars-kb (get-value tcb-selected 'out-vars)))
          (dolist (ov (reverse (slot-value instance 'out-var-mapping)))
            (setf (cdr (assoc (car ov) (slot-value instance 'out-var-mapping))) (pop tcb-out-vars-kb))))

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        ;; generate sync-vars
        ;; copy the a list to not store the values in the library!
        ;; do not push nil on list!
        (dolist (var (copy-list (get-value tcb-selected 'sync-variables)))
          (push `( ,var . ,(make-instance 'tcl-sync-variable :name var)) (slot-value instance 'sync-variables)))
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        ;; execute entry action
        (setf result (execute-action instance (get-value tcb-selected 'action)))))
    
    ;; return
    (cond
      ((equal result '(ERROR (NO TCB MATCHED)))
        ;;(format t "tcb (~s ~s) not matched [0] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result)
        nil)
      ;; wait for sync var case -- TODO what to do in case the TCB also uses an normal EVENT?
      ((not (null (tcb-reg-sync-vars instance)))
        (setf result `(WAIT-FOR-SYNC-VAR ,result))
        (format t "tcb (~s ~s) wait sync var [0] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result))
      ((and (null (slot-value instance 'stack)) (null (slot-value instance 'events)))
        (setf result `(TCB-FINISHED ,result)))
        ;(format t "tcb (~s ~s) finished [0] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result))
      ((and (not (null (tcb-events instance))) (null (tcb-stack instance)))
        (setf result `(WAIT-FOR-EVENT ,result)))
        ;(format t "tcb (~s ~s) wait event [0] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result))
      (T
        (setf result `(OK ,result))))
        ;(format t "tcb (~s ~s) T [0] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result)))

    (setf (slot-value instance 'current-result) result)))



;;; update-slot <instance> <slot> <var> <value>
;;;    description : bind <value> to <var> and store pair in <slot>
;;;                  Depending whether the <var> is already available the pair is updated or added.
;;;    example     : (update-slot *i* 'env-vars ?x 99)
;;;    returns     : '(SUCCESS)
(defmethod update-slot ((instance tcb-execution-class) slot var value)
  (cond ((equal (assoc var (slot-value instance slot)) nil)
          (push (cons var value) (slot-value instance slot)))
        (T
          (setf (cdr (assoc var (slot-value instance slot))) value)))
  '(SUCCESS))



;;; remove-var-from-slot <instance> <slot> <var>
;;;    description : reomve <var> from <slot>
;;;    example     : (remove-var-from-slot *i* 'env-vars ?x)
;;;    returns     : '(SUCCESS)
(defmethod remove-var-from-slot ((instance tcb-execution-class) slot var)
  (let ((tmp (assoc var (slot-value instance slot))))
    (setf (tcb-env-vars instance) (remove tmp (slot-value instance slot))))
  '(SUCCESS)) 



;;; clear-vars <instance> <slot>
;;;    description : clear-vars from <slot>
;;;    example     : (clear-vars *i* 'env-vars)
;;;    returns     : '(SUCCESS)
(defmethod clear-vars ((instance tcb-execution-class) slot)
  (setf (slot-value instance slot) nil)
  '(SUCCESS)) 
         


;;; show <instance>
;;;    description : print the tcb-execution-class
;;;    example     : (show *i*)
;;;    returns     : '(SUCCESS)
(defmethod show ((instance tcb-execution-class))
  (format t "tcb name + instance    : ~s + ~s~%" (tcb-name instance) instance)
  (format t "  in                   : ~s~%" (tcb-in-vars instance))
  (format t "  out                  : ~s~%" (tcb-out-vars instance))
  (format t "  out-var-mapping      : ~s~%" (tcb-out-var-mapping instance))
  (format t "  env                  : ~s~%" (tcb-env-vars instance))
  (format t "  ext-id               : ~s~%" (tcb-ext-run-time-id instance))
  (format t "  tcb stack            : ~s~%" (tcb-stack instance))
  (format t "  rules                : ~s~%" (tcb-rules instance))
  (format t "  sync-variables       : ~s~%" (tcb-sync-variables instance))
  (format t "  tcb-reg-sync-vars    : ~s~%" (tcb-reg-sync-vars instance))
  (format t "  events               : ~s~%" (tcb-events instance))
  (format t "  event-handler        : ~s~%" (tcb-event-handler instance))
  (format t "  current tcb          : ~s~%" (tcb-current instance))
  (format t "  current mode         : ~s~%" (tcb-current-mode instance))
  (format t "  current result       : ~s~%" (tcb-current-result instance))
  (format t "  abort-action         : ~s~%" (tcb-abort-action instance))
  '(SUCCESS))



;;; bind-var <instance> <var> <value>
;;;    description : bind <value> to <var> and store pair in tcb-env-vars
;;;                  Depending whether the <var> is already available the pair is updated or added.
;;;    example     : (bind-var *i* ?x 99)
;;;    returns     : '(SUCCESS)
(defmethod bind-var ((instance tcb-execution-class) var value)
  (update-slot instance 'env-vars var value)
  '(SUCCESS))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; SYNC VAR 
;;; read-sync-variable <instance> <varname>
;;;    description : 
;;;    example     : (read-sync-variable *i* varname)
;;;    returns     :
(defmethod read-sync-variable ((instance tcb-execution-class) name)
  (var-value (cdr (assoc name (tcb-sync-variables instance)))))


;;; reset-sync-variable <instance> <varname> <value>
;;;    description : 
;;;    example     : (write-sync-variable *i* varname value)
;;;    returns     :
(defmethod reset-sync-variable ((instance tcb-execution-class) name)
  (format t "reset-sync-variable - name:~s -  sync-vars:~s ~%" name (tcb-sync-variables instance))
  (let ((tmp (assoc name (tcb-sync-variables instance))))
    (cond
      (tmp
        ;(format t "Found var reset:~s ~%" tmp)
        (reset-var (cdr tmp)))
      (T 
        (format t "Error not able to find var:~s~%" name)
        nil))))


;;; write-sync-variable <instance> <varname> <value>
;;;    description : 
;;;    example     : (write-sync-variable *i* varname value)
;;;    returns     :
(defmethod write-sync-variable ((instance tcb-execution-class) name value)
  (format t "write-sync-variable - name:~s -  sync-vars:~s ~%" name (tcb-sync-variables instance))
  (let ((tmp (assoc name (tcb-sync-variables instance))))
    (cond
      (tmp
        ;(format t "Found var replacing value:~s ~%" tmp)
        (set-value (cdr tmp) value)
        (dolist (i (var-waiting-instances (cdr tmp)))
          (format t "Generate Event for var instance:~s ~%" i)
          (push `(( coordination-interfaces-type . "SYNC-VAR-MOD")( coordination-interface-instance . "SYNC-VAR-MOD.SYNC-CI")( service-name . ,name)( id . ,(car i))( data . ,value)) *SYNC-VAR-EVENTS*))

        (setf (var-waiting-instances (cdr tmp)) nil))
      (T 
        (format t "Error not able to find var:~s~%" name)
        nil))))


;;; wait-for-sync-variable <instance> <varname> <value>
;;;    description : 
;;;    example     : (wait-for-sync-variable *i* varname value)
;;;    returns     : (SUCCESS(NO-WAIT)) (SUCCESS(WAIT)) (ERROR(UNKOWN-VAR))
(defmethod wait-for-sync-variable ((instance tcb-execution-class) evt-name  var-name evt-hndlr)
  (let ((event nil)
        (sync-var-entry (assoc var-name (tcb-sync-variables instance))))
    (format t "wait-for-sync-variable - evt-name: ~s evnt-hndlr: ~s var-name:~s ~%" evt-name evt-hndlr var-name)
    (cond
      (sync-var-entry
        ;(format t "Found var - registering event:~s ~%" sync-var-entry)
        (let ((sync-var (cdr sync-var-entry)))
          (cond
            ; in case the var is already set finish the block execute the handler and all is done.
            ((var-set sync-var)
              (format t "sync-var already set --> execute handler NOW, no wait!~%")
              ;this will execute the event handler right away, it would be better to execute it past the current action block!!
              ;but still better now then never, in this case!
              (let ((result nil)
                    (evt-handler nil))
                (setf evt-handler  (query-kb *MEMORY* '(is-a name) `((is-a sync-var-handler)(name ,evt-hndlr))))
                ;; execute event handler
                (setf result (execute-action instance (get-value evt-handler 'action))))
              '(SUCCESS(NO-WAIT)))
            
            ; regular case the var is not set so far, block the further expansion of the tree in this branch  
            (T
              
              (let ((waiting-instance-id (set-wait sync-var instance)))
                (format t "Generated an activated event for the var- waiting-inst-id:~s ~%" waiting-instance-id)
                (setf event (generate-event *SMARTSOFT* `("SYNC-VAR-MOD" SYNC-VAR-MOD "SYNC-CI" ,var-name ((CONTINUOUS NIL))) ))

                (setf (event-id event) waiting-instance-id)
                (setf (event-state event) 'activated))

             ;push the event on the list of activated events
             (push event *ACTIVATED-EVENT-LIST*)
             ;add the event to the registering tcb
             (update-slot instance 'events evt-name event)
             (update-slot instance 'event-handler evt-name evt-hndlr)
             (update-slot instance 'reg-sync-vars var-name nil)
            '(SUCCESS(WAIT))))))

      (T 
        (format t "Error not able to find var:~s~%" var-name)
        '(ERROR(UNKOWN-VAR))))))


;;; destroy-event-sync-variable <instance> <varname> <value>
;;;    description : 
;;;    example     : (destroy-event-sync-variable *i* varname value)
;;;    returns     :
(defmethod destroy-event-sync-variable ((instance tcb-execution-class) event)
  (format t "destroy event sync-variable ~s ~%" event)
  (let ((tmp (assoc (event-service event) (slot-value instance 'reg-sync-vars))))
    (setf (tcb-reg-sync-vars instance) (remove tmp (slot-value instance 'reg-sync-vars))))
    ;; IMPORTANT IF THE VAR IS NOT REMOVED THE EXECUTION OF THE BLOCK WILL NOT CONTINUE!!!
  ;(show instance)

  ;; use generic method to destory the event!
  (destroy-event instance event)
  '(SUCCESS))

;; SYNC VAR 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  


;;; clear-tcb-stack <instance>
;;;    description : delete the whole tcb stack
;;;    example     : (clear-tcb-stack *i*)
;;;    returns     : '(SUCCESS)
(defmethod clear-tcb-stack ((instance tcb-execution-class))
  (setf (tcb-stack instance) nil)
  '(SUCCESS))



;;; push-tcb <instance> <value>
;;;    description : push a single tcb in front of tcb stack
;;;    example     : (push-tcb *i* 'tcb-1)
;;;    returns     : '(SUCCESS)
(defmethod push-tcb ((instance tcb-execution-class) tcb)
  (format t "push-tcb: ~s --> " (tcb-stack instance))
  (let ((tcb-with-prefix (append-prefix tcb (slot-value instance 'module-inst))))
  (push tcb-with-prefix (tcb-stack instance))
  (format t "~s ~%" (tcb-stack instance)))
  '(SUCCESS))

;;; push-back-tcb <instance> <value>
;;;    description : push a single tcb to the back of the tcb stack
;;;    example     : (push-back-tcb *i* 'tcb-1)
;;;    returns     : '(SUCCESS)
(defmethod push-back-tcb ((instance tcb-execution-class) tcb)
  (format t "push-back-tcb: ~s --> " (tcb-stack instance))
  (let ((tcb-with-prefix (append-prefix tcb (slot-value instance 'module-inst))))
  (cond
    ((tcb-stack instance)
      (push tcb-with-prefix (cdr (last (tcb-stack instance)))))
    (T
      ;(format t "empty plan --> push front ~%")
      (push tcb-with-prefix (tcb-stack instance))))
  (format t "~s ~%" (tcb-stack instance)))
  '(SUCCESS))


;;; pop-tcb <instance> <value>
;;;    description : pop a single tcb from tcb stack
;;;    example     : (pop-tcb *i* 'tcb-1)
;;;    returns     : '(SUCCESS)
(defmethod pop-tcb ((instance tcb-execution-class))
  (pop (tcb-stack instance))
  '(SUCCESS))


;;; push-plan <instance> <value>
;;;    description : push a list of tcb's in front of tcb stack
;;;    example     : (push-plan *i* '(tcb-1 tcb-2))
;;;    returns     : '(SUCCESS)
(defmethod push-plan ((instance tcb-execution-class) tcb-list)
  (let ((tcb-list-with-prefix (append-prefix-to-each-entry tcb-list (slot-value instance 'module-inst))))
  (dolist (tcb (reverse tcb-list-with-prefix))
    (push tcb (tcb-stack instance))))
  '(SUCCESS))

;;; push-back-plan <instance> <value>
;;;    description : push a list of tcb's to the back of tcb stack
;;;    example     : (push--back-plan *i* '(tcb-1 tcb-2))
;;;    returns     : '(SUCCESS)
(defmethod push-back-plan ((instance tcb-execution-class) tcb-list)
  (let ((tcb-list-with-prefix (append-prefix-to-each-entry tcb-list (slot-value instance 'module-inst))))
  (dolist (tcb (reverse tcb-list-with-prefix))
    (cond
    ((tcb-stack instance)
      (push tcb (cdr (last (tcb-stack instance)))))
    (T
      (push tcb (tcb-stack instance))))))
  '(SUCCESS))


;;; activate-event <instance> <evt-name> <evt-hndlr> <server> <service> <mode>
;;;    description : create and activate a event
;;;    example     : (activate-event *i* 'evt-1 'stt 'sttevent 'continuous)
;;;    returns     : '(SUCCESS) '(ERROR (COMMUNICATION))
(defmethod activate-event ((instance tcb-execution-class) evt-name evt-hndlr server service mode param)
  (let ((event nil)
        (module (tcb-module instance))
        (module-inst (tcb-module-inst *CURRENT-INSTANCE*)))
    ;(format t " DEBUG evt-name: ~s evnt-hndlr: ~s server:  ~s service: ~s mode ~s param: ~s ~%" evt-name evt-hndlr server service mode param)
    ;;filter all special events not belonging to external stuff!
    (cond
      ((equal server 'TIMER)
       (setf event (communication *SMARTSOFT* (list 'special 'special server 'event 'generate (list 'TIMER 'TIMER server service mode param)))))
      (T ;;all other stuff uses the regular event generation methods
       (setf event (communication *SMARTSOFT* (list 'special 'special server 'event 'generate (list module module-inst server service mode param))))))
    (cond
      ((null (communication *SMARTSOFT* (list 'special 'special server 'event 'activate (list event))))
       (format t "Error activating Event -->destroy it~%")
       (communication *SMARTSOFT* (list 'special 'special nil 'event 'destroy (list event)))
       '(ERROR (COMMUNICATION)))
      (T
      ;;(push (cons (eval evt-name) event) (tcb-events instance))
        (push event *ACTIVATED-EVENT-LIST*)
        (update-slot instance 'events evt-name event)
        (update-slot instance 'event-handler evt-name evt-hndlr)
        '(SUCCESS)))))

(defmethod activate-event-direct ((instance tcb-execution-class) evt-name evt-hndlr server service mode param module module-inst)
  (let ((event nil))
    ;(format t " DEBUG evt-name: ~s evnt-hndlr: ~s server:  ~s service: ~s mode ~s param: ~s ~%" evt-name evt-hndlr server service mode param)
    ;;filter all special events not belonging to external stuff!
    (cond
      ((equal server 'TIMER)
       (setf event (communication *SMARTSOFT* (list 'special 'special server 'event 'generate (list 'TIMER 'TIMER server service (append (list mode) param))))))
      (T ;;all other stuff uses the regular event generation methods
       (setf event (communication *SMARTSOFT* (list 'special 'special server 'event 'generate (list module module-inst server service 
         (append (list mode) (if (atom param )
                               (list param) 
                               param))))))))
    (cond
      ((null (communication *SMARTSOFT* (list 'special 'special server 'event 'activate (list event))))
       (format t "Error activating Event -->destroy it~%")
       (communication *SMARTSOFT* (list 'special 'special nil 'event 'destroy (list event)))
       '(ERROR (COMMUNICATION)))
      (T
      ;;(push (cons (eval evt-name) event) (tcb-events instance))
        (push event *ACTIVATED-EVENT-LIST*)
        (update-slot instance 'events evt-name event)
        (update-slot instance 'event-handler evt-name evt-hndlr)
        '(SUCCESS)))))

;;; activate-component-life-cycle-event <evt-name> <evt-hndlr> <server> <service>
;;;    description : create and activate a component life cycle event 
;;;    example     : (activate-component-life-cycle-event 'evt-1 'stt 'sttevent)
;;;    returns     : '(SUCCESS)
(defmethod activate-component-life-cycle-event (evt-name evt-hndlr server service)
  (let ((event nil))
    (format t " DEBUG evt-name: ~s evnt-hndlr: ~s server:  ~s service: ~s ~%" evt-name evt-hndlr server service)
    (setf event (communication *SMARTSOFT* (list 'special 'special server 'event 'generate (list *COMPONENT-MODULE-NAME* (intern *COMPONENT-MODULE-NAME*) server service (append (list 'continuous) nil)))))
    (communication *SMARTSOFT* (list 'special 'special server 'event 'activate (list event)))
    (push event *COMPONENT-LIFECYCLE-EVENT-LIST*)
    ;; this handler is not assigned to a specific TCB
;    (update-slot instance 'events evt-name event)
;    (update-slot instance 'event-handler evt-name evt-hndlr)
    '(SUCCESS)))



;;; deactivate-event <instance> <evt-name> <server> <service> <mode>
;;;    description : deactivate and destroy event
;;;    example     : (deactivate-event *i* 'evt-1)
;;;    returns     : '(SUCCESS)
(defmethod destroy-event ((instance tcb-execution-class) evt)
  ;(format t "destroy event ~s ~%" evt)
  (cond
    ((equal (event-module evt) "SYNC-VAR-MOD")
      ;(format t "Destory Sync-var module detected!~%")
      (setf (event-state evt) 'deactivated)
      (destroy-event *SMARTSOFT* (list evt)))
      
    (T
      (communication *SMARTSOFT* (list 'special 'special nil 'event 'deactivate (list evt)))
      (communication *SMARTSOFT* (list 'special 'special nil 'event 'destroy (list evt)))))
  ;;(setf (tcb-event-handler instance) (remove-if #'(lambda (e) (equal (car (assoc) evt)) (tcb-events instance))))  TODO
  (setf (tcb-events instance) (remove-if #'(lambda (e) (equal (cdr e) evt)) (tcb-events instance)))
  (setf *ACTIVATED-EVENT-LIST* (delete evt *ACTIVATED-EVENT-LIST*)) 
  (setf *PENDING-EVENT-LIST* (delete evt *PENDING-EVENT-LIST*))
  '(SUCCESS))



;;; select-current-tcb <instance>
;;;    description : get next tcb/action from tcb stack and put it into current-slot
;;;    example     : (select-current-tcb *i*)
;;;    returns     : (ERROR (NO TCB MATCHED))
;;;                  (OK (TCB SELECTED))
;;;                  (ERROR (STACK IS EMPTY))
;;;                  (ERROR (UNKNOWN TCB))
;;;                  (ERROR ())
(defmethod select-current-tcb ((instance tcb-execution-class))
  ;;take one tcb from stack
  (let ( (result '(ERROR ()))
         (entry-list (pop (tcb-stack instance))))
    (cond
      ;; entry not null --> make instance and put it into current slot
      ((not (null entry-list))
        ;(format t "entry-list ~s ~%" entry-list)
        (cond
          ;; parallel
          ((equal (first entry-list) 'parallel)
            (setf (tcb-current-mode instance) 'parallel)
            (setf entry-list (second entry-list)))
          
          ((equal (first entry-list) 'one-of)
            (setf (tcb-current-mode instance) 'one-of)
            (setf entry-list (second entry-list)))
          
          (T
            (setf entry-list (list entry-list))))
        
        (dolist (entry entry-list)
          ;(format t "make instance ~s ~%" entry)
          (let ((signature nil))
            ;; compose signature
            (multiple-value-bind (name module-inst in-vars out-vars ext-run-time-id)
              (parse-tcl-signature entry)
              ;(format t "select-current-tcb name:~s module-inst:~s in-var:~s out-vars:~s" name module-inst in-vars out-vars)
              (if (not (null module-inst))
                (setf name (intern (concatenate 'string (symbol-name module-inst) "." (symbol-name name)))))
              (cond
                ;; in + out
                ((and (> (length in-vars) 0) (> (length out-vars) 0))
                  (if (not (null ext-run-time-id))
                    (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars instance)) 
                                                        (list '=>) out-vars (list ':ID) ext-run-time-id))
                    (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars instance)) (list '=>) out-vars))))
                ;; in
                ((and (> (length in-vars) 0) (= (length out-vars) 0))
                  (if (not (null ext-run-time-id))
                    (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars instance)) (list ':ID) ext-run-time-id))
                    (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars instance))))))
                ;; out
                ((and (= (length in-vars) 0) (> (length out-vars) 0))
                  (if (not (null ext-run-time-id))
                    (setf signature (append (list name) (list '=>) out-vars (list ':ID) ext-run-time-id))
                    (setf signature (append (list name) (list '=>) out-vars))))
                ;; no in/out
                (T
                  (if (not (null ext-run-time-id))
                    (setf signature (append (list name) (list ':ID) ext-run-time-id))
                    (setf signature (append (list name)))))))
          
            ;; make instance
            ;; push sync-variables to child
            (push (make-instance 'tcb-execution-class :signature signature :parent-sync-variables (tcb-sync-variables instance)) (tcb-current instance)))
        
          ;; check whether tcb could be matched with *MEMORY*
          (cond
            ;; tcb could not be matched
            ((null (tcb-name (first (tcb-current instance))))
              (format t "NO TCB MATCHED~%---------- TODO: EXIT ----------~%~%~%")
              (setf (tcb-current instance) nil)
              (setf result '(ERROR (TCB NOT MATCHED)))
              (return))
          
            ;; tcb selected
            (T
              (setf result '(OK (TCB SELECTED)))))))
      
      ;; stack is empty --> tcb is finished
      (T
        (format t "nothing on stack~%")
        ;;(setf (tcb-current instance) nil)
        (setf result '(ERROR (STACK IS EMPTY)))))
    result))



;;; execute-action <instance> <action>
;;;    description : execute action
;;;    example     :
;;;    returns     : (OK ())
;;;                  (???)                           pass through of return value of meta operator execute
(defmethod execute-action ((instance tcb-execution-class) action)
  (let ((result '(ERROR ())))
    (if *DEDBUG-CI* (format t "~%~%##########################################################################~%"))
    (if *DEDBUG-CI* (format t "execute-action~%"))
    (if *DEDBUG-CI* (format t "instance:     ~a~%" (tcb-name instance)))
    (if *DEDBUG-CI* (format t "module:       ~a~%" (tcb-module instance)))
    (if *DEDBUG-CI* (format t "module-inst:  ~a~%" (tcb-module-inst instance)))
    (if *DEDBUG-CI* (format t "tcb-env-vars: ~a~%" (tcb-env-vars instance)))
    (if *DEDBUG-CI* (format t "##########################################################################~%"))
    (dolist (step action result)
      (cond
        ((equal (first step) 'tcl-bind-var)
          (setf result (eval (list 'tcl-bind-var :name (third step) :value (apply-substitution (fifth step) (tcb-env-vars instance))))))
        
        (T
          (setf result (eval (apply-substitution step (tcb-env-vars instance)))))))
    result))


;;; execute-rule <instance>
;;;    description : search for corredponding rule in *MEMORY*. If matched put rule in current-slot and execute (action) current slot
;;;    example     :
;;;    returns     : (OK (...))
;;;                  (ERROR (NO RULE MATCHED))
;;;                  (ERROR (RULE NOT IN KB))
(defmethod execute-rule ((instance tcb-execution-class) tcb)
  (format t "execute-rule: instance -- ~s ~s   child tcb: ~s ~s result: ~s~%" (tcb-name instance) (tcb-in-vars instance) (tcb-name tcb) (tcb-in-vars tcb) (second (tcb-current-result tcb)))
  (let ((result '(ERROR (NO RULE MATCHED))))
       (dolist (rule-name (tcb-rules instance) result)
         ;(format t "checking rule: ~s ~%" rule-name)
         (let ((rule-instance (query-kb *MEMORY* '(is-a name) `((is-a rule)(name ,rule-name)))))
              (cond
                ; found corresponding rule instance
                ; now try to unify task header and return value
                ((not (null rule-instance))
                  (let ((var-header (unify
                                      (get-value rule-instance 'tcb)
                                      (append (list (tcb-name tcb)) (tcb-in-vars tcb) (list (length (tcb-out-vars tcb))))))
                         (var-result (unify (get-value rule-instance 'return-value) (second (tcb-current-result tcb)))))
                    ;;
                    ;(format t "var-header unify: ~s <--> ~s --> ~s~%"
                    ;  (get-value rule-instance 'tcb)
                    ;  (append (list (tcb-name tcb)) (tcb-in-vars tcb) (list (length (tcb-out-vars tcb))))
                    ;  var-header)
                    ;(format t "var-result unify: ~s <--> ~s --> ~s~%"
                    ;  (get-value rule-instance 'return-value) 
                    ;  (second (tcb-current-result tcb))
                    ;  var-result)

                    ;; MATTHIAS
                  
                    (if (equal (get-value rule-instance 'tcb) '(ANY-TCB 0)) (setf var-header T))
                    (if (equal (get-value rule-instance 'return-value) '(ANY-RESULT)) (setf var-result T))

                    ;(format t "CURRENT RESULT ~s~%" (second (tcb-current-result tcb)))
                    ;(format t "CURRENT HEADER: ~s~%" (append (list (tcb-name tcb)) (tcb-in-vars tcb) (list (length (tcb-out-vars tcb)))))
                    ;(format t "Rule TCB Name: ~s~%" (get-value rule-instance 'tcb))
                    ;(format t "Rule RET Value: ~s~%" (get-value rule-instance 'return-value))

                    ;(format t "var-header ~s~%" var-header)
                    ;(format t "var-result ~s~%" var-result)

                    ;; END MATTHIAS

                    ;;
                    (cond
                      ; rule does NOT match current situation 
                      ((or (equal var-header 'fail) (equal var-result 'fail))
                        nil)
                      
                      ; rule does match current situation
                      (T
                        (format t "  rule ~s matched~%" rule-name)
                        (setf *CURRENT-INSTANCE* instance)
                        (setf *RULE-CHILD-TCB* tcb)
                        (setf result (execute-action instance (get-value rule-instance 'action)))
                        (cond 
                          ((or (equal (first result) 'SUCCESS) (null result))
                            ;; update var-bindings
                            ;;(setf *CURRENT-INSTANCE* instance)
                            (dolist (ovm (reverse (tcb-out-var-mapping tcb)))
                              ;(format t "    - ovm: ~s ~%" ovm)
                              (bind-var instance (car ovm) (cdr (assoc (cdr ovm) (tcb-env-vars tcb)))))))
                        (setf result `(OK ,result))
                        (return)))))
                (T
                  (format t "  rule with name ~s not found in knowledge base~%" rule-name)
                  (setf result '(ERROR (RULE NOT IN KB)))))))
    result))



;;; update-env-apply-rule <instance>
;;;    description : 
;;;    example     :
;;;    returns     : 
(defmethod update-env-apply-rule ((instance tcb-execution-class) tcb)
  (let ( (result '(ERROR ())))
         ;;(tcb (first (tcb-current instance))))
    (cond
      ;; applying a rule is optional
      ((or (equal (first (second (tcb-current-result tcb))) 'SUCCESS) (null (second (tcb-current-result tcb))))
        ;(format t "tcb execution was successfull --> update env variables ~%")
        ;; update var-bindings
        (dolist (ovm (reverse (tcb-out-var-mapping tcb)))
          (bind-var instance (car ovm) (cdr (assoc (cdr ovm) (tcb-env-vars tcb)))))
        ;; apply rule
        (setf result (execute-rule instance tcb))
        ;;(pop (tcb-current instance)) ;; ---> remove
        (setf (tcb-current instance) (remove-if #'(lambda (e) (equal e tcb)) (tcb-current instance)))
        (cond
          ((equal (second result) '(NO RULE MATCHED))
            (setf result nil)))
        (setf result `(TCB-FINISHED ,(second result))))
        
      ;; applying a rule is mandatory
      ((equal (first (second (tcb-current-result tcb))) 'ERROR)

        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; matthias
        ;;(format t "tcb execution was successfull --> update env variables ~%")
        ;; update var-bindings
        (dolist (ovm (reverse (tcb-out-var-mapping tcb)))
          (bind-var instance (car ovm) (cdr (assoc (cdr ovm) (tcb-env-vars tcb)))))
        ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        ;; apply rule
        (setf result (execute-rule instance tcb))
        (cond
          ((equal (first result) 'OK)
            ;;(pop (tcb-current instance)) ;; ---> remove
            (setf (tcb-current instance) (remove-if #'(lambda (e) (equal e tcb)) (tcb-current instance)))
            (setf result `(TCB-FINISHED ,(second result))))
          (T
            (setf result '(ERROR (NO RULE MATCHED)))
            (format t "~%~%NO RULE IN ERROR CASE  -- ~s ~s result: ~s ~%~%" (tcb-name tcb) (tcb-in-vars tcb) result))))
                    
      (T
        (format t " ERROR 1 RULE NOT SUCCESS or ERROR ~%~%~%~%~%~%~%~%")
        ))
    result))




;;; execute-one-step <instance>
;;;    description : 
;;;    example     :
;;;    returns     : (OK (???))
;;;                  (STACK-EMPTY (???))
;;;                  
(defmethod execute-one-step ((instance tcb-execution-class))
  ;(format t "execute one step ------ ~s -------------------------~%" (tcb-name instance))
  ;(show instance)
  ;(format t "-------------------------------------------------~%")
  (setf *CURRENT-INSTANCE* instance)
  (let ( (result '(OK (NOTHING TO DO))))
    (cond
      ((and (null (tcb-current instance)) (null (tcb-stack instance)) (null (tcb-events instance)))
        (format t "~%~%~%ERROR - THIS SHOULD NOT BE EXECUTED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ~%~%~%~%")
        (setf result '(TCB-FINISHED ())))
      
      (T
        ;; start event managenment

	;; COMPONENT LIFECYCLE EVENTS
	(cond
          ((not (null *PENDING-EVENT-LIST*))
	    (dolist (evt *PENDING-EVENT-LIST*)
              (cond 
                ((equal (event-module evt) "COMPONENT")
                  ;(format t "Matched Component Event: ~s~%" evt)
                  (cond
                    ((not (null (check evt))) ;;this should never fail!
                      ;;(format t "execute event handler: ~s~%" msg)
                      (let ((evt-handler  (query-kb *MEMORY* '(is-a is-lifecycle-event) '((is-a event-handler)(is-lifecycle-event T)))))
                        ;;(format t "handler-action: ~s ~%" (get-value evt-handler 'action))
                        (setf *EVT* evt)
                        (setf *MSG* (getmessage evt))
                        ;; execute event handler
                        (setf result (execute-action instance (get-value evt-handler 'action)))

                        (format t "after event-handler execution~%")
                        (setf *EVT* nil)
                        (setf *MSG* nil)))))))))

	;; EVENT PORT EVENTS
        (cond
          ((not (null (tcb-events instance)))
            (if *DEDBUG-CI* (format t "process event in tcb: ~s ~s ~%" (tcb-name instance) (tcb-in-vars instance)))
            (if *DEDBUG-CI* (format t "process event tcb-events : ~s ~%" (tcb-events instance)))

            (setf result '(WAIT-FOR-EVENT ()))
            (dolist (evt (tcb-events instance))
              ;;(let ((msg (getmessage (cdr evt))))
               ;(format t "(check (cdr evt)): ~s~%" (check (cdr evt)))
                (cond
                  ;;((not (null msg))
                  ((not (null (check (cdr evt))))
                    ;(format t "Event matched evt:~s ~%" evt)
                    ;(format t "Event matched assoc cdr ~s ~%" (cdr(assoc (car evt) (tcb-event-handler instance))))
                    
                    (let ((evt-handler  (query-kb *MEMORY* '(is-a name) `((is-a event-handler)(name ,(cdr(assoc (car evt) (tcb-event-handler instance))))))))
                      (cond
                        ((equal (event-module (cdr evt)) "SYNC-VAR-MOD")
                          (format t "Sync Var event handler!~%")
                          (setf evt-handler  (query-kb *MEMORY* '(is-a name) `((is-a sync-var-handler)(name ,(cdr(assoc (car evt) (tcb-event-handler instance)))))))
                           ;(format t "handler-action: ~s ~%" (get-value evt-handler 'action))
                           (setf *EVT* (cdr evt))
                           (setf *MSG* (getmessage (cdr evt)))
                           ;; execute event handler
                           (setf result (execute-action instance (get-value evt-handler 'action)))
                           (destroy-event-sync-variable instance (cdr evt)))

                        ;; normal event handler case!
                        (T
                         ;(format t "handler-action: ~s ~%" (get-value evt-handler 'action))
                         (setf *EVT* (cdr evt))
                         (setf *MSG* (getmessage (cdr evt)))
                         ;; execute event handler
                         (setf result (execute-action instance (get-value evt-handler 'action)))))

                      ;(format t "after event-handler execution~%")
                      (setf *EVT* nil)
                      (setf *MSG* nil)
                      
                      ;; next step -- abort / finish / wait-for-event
                      (cond
                        ;; no active events --> abort child tcb's
                        ((null (tcb-events instance))
                          (cond
                            ((not (null (tcb-current instance)))
                              (abort-tcb-all instance)))
                          (setf result `(TCB-FINISHED ,result)))
                          ;(format t "tcb (~s ~s) finished [1] result: ~s~%" (tcb-name instance) (tcb-in-vars instance) result))
                        
                        ;; wait for events
                        (T
                          (setf result `(WAIT-FOR-EVENT ,(tcb-current-result instance))))))
                          ;(format t "wait for event [1] (~s ~s) result: ~s ~%" (tcb-name instance) (tcb-in-vars instance) result))))
                    (return))))))
        ;; end event management
        
        
        (cond
          ((not (equal (first result) 'TCB-FINISHED))
          
           ;(format t "[~s] tcb-current:~s tcb-stack:~s result :~s~%" (tcb-name instance) (tcb-current instance) (tcb-stack instance) result)
            (cond
              ;; either wait for sync var or 
              ;((equal (first result) 'WAIT-FOR-SYNC-VAR)
              ((not (null (tcb-reg-sync-vars instance)))
                (format t "[~s] wait for sync-var~%" (tcb-name instance))
                (setf result '(WAIT-FOR-EVENT())))

              ;; select new tcb
              ((and (null (tcb-current instance)) (not (null (tcb-stack instance))))
                ;(format t "select new tcb~%")
                (select-current-tcb instance)
                ;;(let ((tcb (first (tcb-current instance))))
                (let ((overall-result nil))
                  (dolist (tcb (tcb-current instance))
                    (cond
                      ;; tcb is finished
                      ((equal (first (tcb-current-result tcb)) 'TCB-FINISHED)
                        ;;update variable bindings and apply rules
                        ;(format t "child tcb (~s ~s) finished ~%" (tcb-name tcb) (tcb-in-vars tcb))
                        (setf *CURRENT-INSTANCE* instance)
                        (setf result (update-env-apply-rule instance tcb)))
            
                      ((equal (first (tcb-current-result tcb)) 'WAIT-FOR-EVENT)
                        (if (equal (first overall-result) 'OK) 
                          nil
                          (setf result '(WAIT-FOR-EVENT()))))
                
                      (T
                        (setf overall-result '(OK ()))
                        (setf result '(OK ())))))))
                
                
              ;; or execute current slot
              ((not (null (tcb-current instance)))
                ;;(let ((tcb (first (tcb-current instance))))
                (let ((overall-result nil))
                  (dolist (tcb (reverse (tcb-current instance)))
                    ;(format t "execute child tcb ~s ~s~%" (tcb-name tcb) (tcb-in-vars tcb))
                    (setf result (execute-one-step tcb))
                    (cond
                      ;; tcb is finished
                      ((equal (first (tcb-current-result tcb)) 'TCB-FINISHED)
                        ;;update variable bindings and apply rules
                        ;(format t "child tcb (~s ~s) finished ~%" (tcb-name tcb) (tcb-in-vars tcb))
                        (setf *CURRENT-INSTANCE* instance)
                        (setf result (update-env-apply-rule instance tcb))
                        ;; if one-of --> abort all
                        (cond
                          ((equal (tcb-current-mode instance) 'one-of)
                            (abort-tcb-all instance)
                            (return))))
            
                      ((equal (first (tcb-current-result tcb)) 'WAIT-FOR-EVENT)
                        (if (equal (first overall-result) 'OK) 
                          nil
                          (setf result '(WAIT-FOR-EVENT())))) ;; else
                
                      (T
                        (setf overall-result '(OK ()))
                        (setf result '(OK ()))))))))))
        
        
        ;; return
        ;(format t "start return ... result: ~s~%" result)
        (cond
          ((equal (first result) 'WAIT-FOR-EVENT)
            (setf result '(WAIT-FOR-EVENT ())))
            ;(format t "wait for event [3a] (~s ~s) result: ~s ~%" (tcb-name instance) (tcb-in-vars instance) result))
          
          ;; tcb is finished --> pass through
          ((and (null (tcb-stack instance)) (null (tcb-current instance)))
            (cond
              ((null (tcb-events instance))
                (setf result `(TCB-FINISHED ,(second result))))
                ;(format t "tcb (~s ~s) finished [3a] result: ~s ~%" (tcb-name instance) (tcb-in-vars instance) result))
              
              (T
                (setf result '(WAIT-FOR-EVENT ())))))
                ;(format t "wait for event [3b] (~s ~s) result: ~s ~%" (tcb-name instance) (tcb-in-vars instance) result))))
    
          ;; else
          (T
            (setf result '(OK ()))))
            ;(format t "T [3a] (~s ~s) result: ~s ~%" (tcb-name instance) (tcb-in-vars instance) result)))
        
        (format t "===stop return (~s) ... result:~s ~%" (tcb-name instance) result)))
    
    (setf (tcb-current-result instance) result)
    result))



;;; abort-tcb <instance>
;;;    description : 
;;;    example     :
;;;    returns     : 
(defmethod abort-tcb ((instance tcb-execution-class))
  (let ((result nil)
        (calling-tcb-instance *CURRENT-INSTANCE*)) ;; save the calling context TCB
    ;; abort child tcb if min one exists
    (cond
      ((not (null (tcb-current instance)))
        (loop
          (let ((tcb (pop (tcb-current instance))))
            (format t "abort-tcb: ~s ~s~%" (tcb-name tcb) (tcb-in-vars instance))
            (abort-tcb tcb)
            (if (null (tcb-current instance)) (return))))))

    ;; destroy events
    (dolist (evt (tcb-events instance))
      (destroy-event instance (cdr evt)))
    ;; delete stack
    (setf (tcb-stack instance) nil)
    ;; execute abort-action
    ;; set the context for the actions to be performed in
    (setf *CURRENT-INSTANCE* instance)
    (setf result (execute-action instance (tcb-abort-action instance)))
    ;; maybe do something with result !!!!!!!!!!!!!! TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ;; set-result
    (setf (tcb-current-result instance) '(ABORTED))
    ;; reset the context TCB
    (setf *CURRENT-INSTANCE* calling-tcb-instance)
    '(SUCCESS)))



;;; abort-tcb-all <instance>
;;;    description : 
;;;    example     :
;;;    returns     : 
(defmethod abort-tcb-all ((instance tcb-execution-class))
  (cond
    ((not (null (tcb-current instance)))
      (loop
        (let ((tcb (pop (tcb-current instance))))
          (format t "abort-tcb-all: ~s ~s~%" (tcb-name tcb) (tcb-in-vars tcb))
          (abort-tcb tcb)
          (if (null (tcb-current instance)) (return)))))))


;;; interrupt-tcb-all <instance>
;;;    description : abort child tcbs and return plan with yet to be finshed tcbs
;;;    example     : TODO THIS IS ONLY A PROTOTYPE AND IS NOT WORKING FOR ALL CASES!!!! (e.g. parallel)
;;;    returns     : 

(defmethod interrupt-tcb ((instance tcb-execution-class))
  (let ((child-stack nil)
        (child-plan nil))
    ;; abort child tcb if min one exists
(show instance)
    (cond
      ((not (null (tcb-current instance)))
        (loop
          (let ((tcb (pop (tcb-current instance))))
            (push tcb child-stack)
            (format t "abort-tcb: ~s ~s~%" (tcb-name tcb) (tcb-in-vars instance))
            (abort-tcb tcb)
            (if (null (tcb-current instance)) (return))))))
    (format t "child-stack ~a~%" child-stack)
    (dolist (tcb child-stack)
      ;(format t "tcb-name ~a~%" (tcb-name tcb))
      ;(format t "tcb-in-var ~a~%" (tcb-in-vars tcb))
      ;(format t "tcb-env-var ~a~%" (tcb-env-vars tcb))

      (let ((signature nil) (name (tcb-name tcb)) (in-vars (tcb-in-vars tcb)) (out-vars  (tcb-out-vars tcb)))
      ;; compose signature
        (cond
          ;; in + out
          ((and (> (length in-vars) 0) (> (length out-vars) 0))
            (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars tcb)) (list '=>) out-vars)))
          ;; in
          ((and (> (length in-vars) 0) (= (length out-vars) 0))
            (setf signature (append (list name) (apply-substitution in-vars (tcb-env-vars tcb)))))
          ;; out
          ((and (= (length in-vars) 0) (> (length out-vars) 0))
            (setf signature (append (list name) (list '=>) out-vars)))
          ;; no in/out
          (T
            (setf signature (append (list name)))))
          ;(format t "signature ~a~%" signature)
          (push signature child-plan)))

    ;; destroy events
    ;(dolist (evt (tcb-events instance))
    ;  (destroy-event instance (cdr evt)))
    ;; save stack
    (setf child-plan (append child-plan  (tcb-stack instance)))
    ;; delete stack
    (setf (tcb-stack instance) nil)

    ;(format t "child-plan ~a~%" child-plan)

    ;; execute abort-action
    ;(setf result (execute-action instance (tcb-abort-action instance)))
    ;; maybe do something with result !!!!!!!!!!!!!! TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ;; set-result
    ;(setf (tcb-current-result instance) '(ABORTED))
    child-plan))



(defun execute (tcb)
  (setf *ACTIVATED-EVENT-LIST* *COMPONENT-LIFECYCLE-EVENT-LIST*)
  (format t "*ACTIVATED-EVENT-LIST* init : ~a~%" *ACTIVATED-EVENT-LIST*)
  (setf *PENDING-EVENT-LIST* nil)
  (setf *CURRENT-INSTANCE* nil)
  (setf *EVT* nil)
  (setf *MSG* nil)
  (let* (
         (tcb-exec (make-instance 'tcb-execution-class :signature tcb :parent-sync-variables nil))
         (result (tcb-current-result tcb-exec)))
    (loop
      (format t "~%~%~%~%~%---------------------------- L O O P    E X E C U T I O N -----------------------------~%")
      ;;(sleep 1)
      (cond
        ((equal (first result) 'TCB-FINISHED)
          (format t "~%~%--------------------------------------------------------------------------")
          (format t "~%---------------------------- F I N I S H E D -----------------------------")
          (format t "~%--------------------------------------------------------------------------")
          (format t "~%--------------------------------------------------------------------------")
          (format t "~%##########################################################################~%~%~%~%")
          (return))
        ((equal (first result) 'WAIT-FOR-EVENT)
          (format t "*ACTIVATED-EVENT-LIST* : ~a~%" *ACTIVATED-EVENT-LIST*)
          (setf *PENDING-EVENT-LIST* (check-events-wait *SMARTSOFT* *ACTIVATED-EVENT-LIST*))
          ;(format t "MATTHIAS peding-event-list : ~s ~%" *PENDING-EVENT-LIST*)
          ;(show (first *PENDING-EVENT-LIST*))
          (setf result (execute-one-step tcb-exec)))
        (T
          ;; check-event
          (cond
            ((not (null *ACTIVATED-EVENT-LIST*))
              (format t "*ACTIVATED-EVENT-LIST* 2 : ~a~%" *ACTIVATED-EVENT-LIST*)
              (setf *PENDING-EVENT-LIST* (check-events *SMARTSOFT* *ACTIVATED-EVENT-LIST*))))
;              (format t "MATTHIAS peding-event-list 2 : ~s ~%" *PENDING-EVENT-LIST*)))
          (setf result (execute-one-step tcb-exec)))))))

     
