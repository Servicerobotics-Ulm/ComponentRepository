;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2010 Andreas Steck
;;
;;		steck@hs-ulm.de
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

;; THIS WILL AVOID A WARINING FOR A UNKOWN FUNCTION - compile-and-load-smarttcl, from load-tcl.lisp
(declaim (ftype (function (&optional string string list list) t) compile-and-load-smarttcl))

(let ((indexpos nil)
      (tcl-prefix nil)
      (module-path nil)
      (lispinterface-prefix ""))
  (setf indexpos  (position "--tcl-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf tcl-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (setf indexpos  (position "--module-path" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf module-path (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (setf indexpos  (position "--lispinterface-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf lispinterface-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (format t "all: ~s~%" sb-ext:*posix-argv*)
  (format t "tcl-Prefix: ~a~%" tcl-prefix)
  (format t "module-path: ~a~%" module-path)
  (format t "lispinterface-prefix: ~a~%" lispinterface-prefix)

  ;; load SmartTCL
  (load (format nil "~asmartTCL/load-tcl.lisp" tcl-prefix))
  (compile-and-load-smarttcl tcl-prefix lispinterface-prefix)


  (format t "~%~%")
  (format t " --------------------------- ~%")
  (format t " load-smartTCL-only finished ~%")
  (format t " --------------------------- ~%~%")

  (read-coordination-module-system-file (format nil "~a/connections-ci-interface-test.json" tcl-prefix))

  (load-coordination-interface "KBCoordinationService" module-path)
  (load-coordination-interface "TestCoordinationService" module-path)
  (load-coordination-interface "TestCoordinationService2" module-path)

  (instantiate-coordination-module "CoordinationTestModule" 'testInst)
  (instantiate-coordination-module "CoordinationTestModule2" 'testInst2)
  (instantiate-coordination-module "KBModule" 'kbModInst))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TESTMODULE


(define-rule (test-rule)
  (tcb (tcb-sub-block-error))
  (return-value (ERROR (TEST)))
  (action (
              (format t "=========================>>> RULE: test-rule (ERROR (TEST))  ~%"))))

(define-event-handler (test-event-handler)
	( action (
		(format t "Test Event handler called ~s~%"(tcl-event-message))
		(tcl-state :server 'test :state "Neutral")
		(tcl-abort))))

(realize-tcb (tcb-testCoordinationInterface  ?x1 ?x2)
	(module "CoordinationTestModule")
	(rules (test-rule))
	
	(action (	
                  (format t "ACTION TEST~%")	
                  ;;SEND
                  (tcl-send :server 'test :service 'testSend :param 1)	
                  ;;QUERY
                  (format t "got Query answer in TCL ~a~%" (tcl-query :server 'test :service 'testQuery :request 22))
                  ;;PARAM
                  (tcl-param :server 'test :slot 'DomainTestObjects.TestParameterSet.TestTrigger)
                  ;;STATE
                  (tcl-state :server 'test :state "Active")
                  ;;EVENT
                  (tcl-activate-event :handler 'test-event-handler :mode 'single :name 'eventname 
                    :server 'test :service 'testEvent)

                  ;;KB
		  (tcl-kb-update :key '(is-a name)  :value '((is-a test) (name test1)))
                  (format t "KB Query answer: ~a~%" (get-value (tcl-kb-query :key '(is-a)  :value '((is-a test) )) 'name))

                  (tcl-push-plan :plan `((tcb-sub-block-error)
                                         (testInst2.tcb-sub-block-handler))))))


(realize-tcb (tcb-sub-block-error)
  (module "CoordinationTestModule")
  (action (	
            (format t "tcb-sub-block~%")
            '(ERROR (TEST)))))



(define-event-handler (test-event-handler2)
	( action (
		(format t "Test Event 2 handler called ~s~%"(tcl-event-message))
		(tcl-state :server 'test2 :state "Neutral")
		(tcl-abort))))

(realize-tcb (tcb-sub-block-handler)
  (module "CoordinationTestModule2")
  (abort-action ( (format t "tcb-sub-block-handler - abort action!~%")
                  (tcl-state :server 'test2 :state "Neutral")))
  (action (	
            (format t "tcb-sub-block-handler~%")
            ;;EVENT
            (tcl-activate-event :handler 'test-event-handler2 :mode 'single :name 'eventname2 
              :server 'test2 :service 'testEvent))))




;; the following three blocks feature the same name, but in different coordination modules!
(realize-tcb (tcb-same-name)
  (action ((format t "tcb-same-name 0~%"))))

(realize-tcb (tcb-same-name)
  (module "CoordinationTestModule")
  (action ((format t "tcb-same-name 1~%"))))

(realize-tcb (tcb-same-name)
  (module "CoordinationTestModule2")
  (action ((format t "tcb-same-name 2~%"))))




(execute '(testInst.tcb-testCoordinationInterface  1 2))

(execute '(tcb-same-name))
(execute '(testInst.tcb-same-name))
(execute '(testInst2.tcb-same-name))

;; error cases! --> this should map to a fallback!
;(execute '(FALSEMODULE.tcb-same-name))
;(execute '(FALSEMODULE.tcb-testCoordinationInterface  1 2))
;(execute '(tcb-testCoordinationInterface  1 2))

;;TEST the default
;(execute '(tcb-testCoordinationInterface  1 2))

;; TESTMODULE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;(defun wait-for-component-shutdown ()
; (format t "call wait for componentshutdown~%")
; (waitoncompshutdown)
; (format t "chomponent is SHUTING DOWN!~%")
; (format t "[main] join-thread:  component... ~%")
;                (waitoncomptasktocomplete)
;                (format t "[main] ...DONE ~%")
;                (format t "QUIT.~%")
;                (exit))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;(format t "Start wait-for-component-shutdown thread~%")
;(setf *wait-shutdown-component-thread* (SB-THREAD:make-thread #'wait-for-component-shutdown))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
;  (shutdowncomp)
;
; ;(format t "[main] join-thread:  check-events-loop ... ~%")
; ;(SB-THREAD:join-thread *wait-shutdown-component-thread*)
; ;(format t "[main] ...DONE ~%")
