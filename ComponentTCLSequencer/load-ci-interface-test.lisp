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

  (instantiate-coordination-module "CoordinationTestModule" 'testInst)
  (instantiate-coordination-module "KBModule" 'kbModInst))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TESTMODULE

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
           )))

(execute '(testInst.tcb-testCoordinationInterface  1 2))


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
