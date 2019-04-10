;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011 Andreas Steck
;;			2015-2016 Matthias Lutz
;;
;;		steck@hs-ulm.de
;;		lutz@hs-ulm.de
;;
;;      Service Robotics Ulm
;;      Christian Schlegel
;;	of Applied Sciences
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

(defun show-tcbs ()
 (show-tcl-library))

(defun show-tcl-library ()
  (let ((tcb-list     (query-kb-all *MEMORY* '(is-a) '((is-a tcb)))))
    (dolist (tcb tcb-list)
      (format t "~%--------------------------------~%")
      (format t "name             : ~s ~%" (get-value tcb 'name))
      (format t "  module         : ~a ~%" (get-value tcb 'module))
      (format t "  module-inst    : ~a ~%" (get-value tcb 'module-inst))
      (format t "  highlevel-flag : ~a ~%" (get-value tcb 'highlevel-flag))
      (format t "  avail-flag     : ~s ~%" (get-value tcb 'avail-flag))
      (format t "  priority       : ~s ~%" (get-value tcb 'priority))
      (format t "  in-vars        : ~s ~%" (get-value tcb 'in-vars))
      (format t "  out-vars       : ~s ~%" (get-value tcb 'out-vars))
      (format t "  in-out         : ~s ~%" (get-value tcb 'in-out))
      (format t "  precondition   : ~s ~%" (get-value tcb 'precondition))
      (format t "  action         : ~s ~%" (get-value tcb 'action))
      (format t "  abort-action   : ~s ~%" (get-value tcb 'abort-action))
      (format t "  plan           : ~s ~%" (get-value tcb 'plan))
      (format t "  rules          : ~s ~%" (get-value tcb 'rules))
      (format t "  sync-vars      : ~s ~%" (get-value tcb 'sync-variables)))))


(defun show-tcl-event-handler ()
  (let ((tcb-list     (query-kb-all *MEMORY* '(is-a) '((is-a event-handler)))))
    (dolist (tcb tcb-list)
      (format t "~%--------------------------------~%")
      (format t "name        : ~s ~%" (get-value tcb 'name))
      (format t "  action    : ~s ~%" (get-value tcb 'action)))))


(defun show-tcl-rules ()
  (let ((tcb-list     (query-kb-all *MEMORY* '(is-a) '((is-a rule)))))
    (dolist (tcb tcb-list)
      (format t "~%--------------------------------~%")
      (format t "name           : ~s ~%" (get-value tcb 'name))
      (format t "  tcb          : ~s ~%" (get-value tcb 'tcb))
      (format t "  return-value : ~s ~%" (get-value tcb 'return-value))
      (format t "  action       : ~s ~%" (get-value tcb 'action)))))

(defmacro REALIZE-TCB (&rest sections)
  (let* ( (tcb-signature (first sections))
          (precondition  (assoc 'precondition   (delete 'sections sections) :test #'equal))
          (priority      (assoc 'priority       (delete 'sections sections) :test #'equal))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal))
          (abort-action  (assoc 'abort-action   (delete 'sections sections) :test #'equal))
          (plan          (assoc 'plan           (delete 'sections sections) :test #'equal))
          (rules         (assoc 'rules          (delete 'sections sections) :test #'equal))
          (module        (assoc 'module         (delete 'sections sections) :test #'equal))
)
    (multiple-value-bind (name inst-name in-vars out-vars)
      (parse-tcl-signature tcb-signature)

      (cond 
        ((not (null inst-name))
          (format t "WARNING: DEFINE-TCB called with module instance name --> ignoring inst-name!~%")))
      (cond 
        ((null priority)
          (setf priority 0))
        (T
          (setf priority (second priority))))
      
      ;; add TCB to KB --> SIGNATURE name in-vars out-vars precondition priority !!!!!!!!!!!!
      (update-kb *MEMORY* '(is-a name in-vars out-vars precondition priority)
        `( (is-a tcb)
           (name ,name)
           (avail-flag ,t)
           (priority ,priority)
           (in-vars ,in-vars)
           (out-vars ,out-vars)
           (in-out ,(list (length in-vars) (length out-vars)))
           (precondition ,(second precondition))
           (rules ,(second rules))
           (action ,(second action))
           (abort-action ,(second abort-action))
           (module ,(second module))
           (plan ,(second plan))))))
  T)


(defmacro DEFINE-LOWLEVEL-TCB (&rest sections)
  (let* ( (tcb-signature (first sections))
          (precondition  (assoc 'precondition   (delete 'sections sections) :test #'equal))
          (priority      (assoc 'priority       (delete 'sections sections) :test #'equal))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal))
          (abort-action  (assoc 'abort-action   (delete 'sections sections) :test #'equal))
          (plan          (assoc 'plan           (delete 'sections sections) :test #'equal))
          (rules         (assoc 'rules          (delete 'sections sections) :test #'equal))
          (module        (assoc 'module         (delete 'sections sections) :test #'equal))
)
    (multiple-value-bind (name inst-name in-vars out-vars)
      (parse-tcl-signature tcb-signature)

      (cond 
        ((not (null inst-name))
          (format t "WARNING: DEFINE-TCB called with module instance name --> ignoring inst-name!~%")))
      (cond 
        ((null priority)
          (setf priority 0))
        (T
          (setf priority (second priority))))
      
      ;; add TCB to KB --> SIGNATURE name in-vars out-vars precondition priority !!!!!!!!!!!!
      (update-kb *MEMORY* '(is-a name in-vars out-vars precondition priority)
        `( (is-a tcb)
           (name ,name)
           (avail-flag ,t)
           (priority ,priority)
           (in-vars ,in-vars)
           (out-vars ,out-vars)
           (in-out ,(list (length in-vars) (length out-vars)))
           (precondition ,(second precondition))
           (rules ,(second rules))
           (action ,(second action))
           (abort-action ,(second abort-action))
           (module ,(second module))
           (plan ,(second plan))))))
  T)

(defmacro DEFINE-HIGHLEVEL-TCB (&rest sections)
  (let* ( (tcb-signature (first sections))
          (precondition  (assoc 'precondition   (delete 'sections sections) :test #'equal))
          (priority      (assoc 'priority       (delete 'sections sections) :test #'equal))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal))
          (abort-action  (assoc 'abort-action   (delete 'sections sections) :test #'equal))
          (plan          (assoc 'plan           (delete 'sections sections) :test #'equal))
          (rules         (assoc 'rules          (delete 'sections sections) :test #'equal))
          (module        (assoc 'module         (delete 'sections sections) :test #'equal))
)
    (multiple-value-bind (name inst-name in-vars out-vars)
      (parse-tcl-signature tcb-signature)

      (cond 
        ((not (null inst-name))
          (format t "WARNING: DEFINE-TCB called with module instance name --> ignoring inst-name!~%")))
      (cond 
        ((null priority)
          (setf priority 0))
        (T
          (setf priority (second priority))))
      
      ;; add TCB to KB --> SIGNATURE name in-vars out-vars precondition priority !!!!!!!!!!!!
      (update-kb *MEMORY* '(is-a name in-vars out-vars precondition priority)
        `( (is-a tcb)
           (name ,name)
           (avail-flag ,t)
           (priority ,priority)
           (in-vars ,in-vars)
           (out-vars ,out-vars)
           (in-out ,(list (length in-vars) (length out-vars)))
           (precondition ,(second precondition))
           (rules ,(second rules))
           (action ,(second action))
           (abort-action ,(second abort-action))
           (module ,(second module))
           (plan ,(second plan))))))
  T)

;; used only for backward compatibility
(defmacro DEFINE-TCB (&rest sections)
  (let* ( (tcb-signature (first sections))
          (precondition  (assoc 'precondition   (delete 'sections sections) :test #'equal))
          (priority      (assoc 'priority       (delete 'sections sections) :test #'equal))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal))
          (abort-action  (assoc 'abort-action   (delete 'sections sections) :test #'equal))
          (plan          (assoc 'plan           (delete 'sections sections) :test #'equal))
          (rules         (assoc 'rules          (delete 'sections sections) :test #'equal))
          (module        (assoc 'module         (delete 'sections sections) :test #'equal))
          (sync-variables (assoc 'sync-variables          (delete 'sections sections) :test #'equal))
)
    (multiple-value-bind (name inst-name in-vars out-vars)
      (parse-tcl-signature tcb-signature)

      (cond 
        ((not (null inst-name))
          (format t "WARNING: DEFINE-TCB called with module instance name --> ignoring inst-name!~%")))
      (cond 
        ((null priority)
          (setf priority 0))
        (T
          (setf priority (second priority))))
      
      ;; add TCB to KB --> SIGNATURE name in-vars out-vars precondition priority !!!!!!!!!!!!
      (update-kb *MEMORY* '(is-a name in-vars out-vars precondition priority)
        `( (is-a tcb)
           (name ,name)
           (avail-flag ,t)
           (priority ,priority)
           (in-vars ,in-vars)
           (out-vars ,out-vars)
           (in-out ,(list (length in-vars) (length out-vars)))
           (precondition ,(second precondition))
           (rules ,(second rules))
           (sync-variables ,(second sync-variables))
           (action ,(second action))
           (abort-action ,(second abort-action))
           (module ,(second module))
           (plan ,(second plan))))))
  T)



(defmacro DEFINE-RULE (&rest sections)
  (let* (
          (rule-name (first sections))
          (tcb-signature (assoc 'tcb            (delete 'sections sections) :test #'equal))
          (return-value  (assoc 'return-value   (delete 'sections sections) :test #'equal))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal)))
    
    (multiple-value-bind (name inst-name in-vars out-vars)
      (parse-tcl-signature (second tcb-signature))

      (cond 
        ((not (null inst-name))
          (format t "WARNING: DEFINE-RULE called with module instance name --> ignoring inst-name!~%")))
      
      ;; add RULE to KB
      (update-kb *MEMORY* '(is-a name)
        `( (is-a rule)
           (name ,(first rule-name))
           (tcb ,(append (list name) in-vars (list (length out-vars))))
           (return-value ,(second return-value))
           (action ,(second action))))))
  T)

(defmacro DEFINE-SYNC-HANDLER (&rest sections)
  (let* (
          (handler-name (first sections))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal)))
    
    ;; add SYNC-HANDLER to KB
    (update-kb *MEMORY* '(is-a name)
      `( (is-a sync-var-handler)
         (name ,(first handler-name))
         (action ,(second action)))))
  T)



(defmacro DEFINE-EVENT-HANDLER (&rest sections)
  (let* (
          (handler-name (first sections))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal)))
    
    ;; add EVENT-HANDLER to KB
    (update-kb *MEMORY* '(is-a name)
      `( (is-a event-handler)
         (name ,(first handler-name))
         (action ,(second action)))))
  T)
          
    
(defmacro DEFINE-COMPONENT-LIFECYCLE-EVENT-HANDLER (&rest sections)
  (let* (
          (handler-name (first sections))
          (action        (assoc 'action         (delete 'sections sections) :test #'equal)))
    
    ;; add EVENT-HANDLER to KB
    (update-kb *MEMORY* '(is-a name)
      `( (is-a event-handler)
         (name ,(first handler-name))
         (is-lifecycle-event T)
         (action ,(second action))))
    (format t "=========================>>> DEFINE COMPONENT LIFECYCLE EVENT HANDLER ~%")
    (activate-component-life-cycle-event 'evt-component-lifecycle
                              (first handler-name) 
                              'component
                              'shutdownEvent)
)
  T)      
          
          
          
       

