;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011 Andreas Steck
;;
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;		of Applied Sciences
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



;; THIS WILL AVOID A WARINING FOR A UNKOWN FUNCTION - initialize, from smartInterface.lisp
(declaim (ftype (function (T) (VALUES (ALIEN INT) &OPTIONAL)) initialize))


(defvar *LISP-INTERFACE-PREFIX* nil)


;load asdf
(require "asdf")
; cffi
(require "cffi")


;; compile and load
(defun compile-and-load-smarttcl ( &optional (tcl-prefix "") (lispinterface-prefix "") (pre-component-startup-function nil pre-component-startup-function-supplied-p) 
                                                                           (pre-component-startup-function-param nil pre-component-startup-function-param-supplied-p) )


  ;; cl-json
  (setf asdf:*central-registry*
       (list* '*default-pathname-defaults*
              (format NIL "~a/cl-json/" tcl-prefix)
              asdf:*central-registry*))
  (require "cl-json")


  ;; load smartSimpleKB
  (compile-file (format nil "~asmartTCL/smartSimpleKB/smartSimpleKB.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/smartSimpleKB/smartSimpleKB.fasl" tcl-prefix))

  ;; Robot Memory
  (defvar *MEMORY* (make-instance 'kb-main-class))

  (compile-file (format nil "~asmartTCL/defs.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/defs.fasl" tcl-prefix))




  (setf *LISP-INTERFACE-PREFIX* lispinterface-prefix)
  (if (equal lispinterface-prefix "")
    (progn
      (compile-file (format nil "~a/smartsoft/src/lispInterface/smartInterface.lisp" tcl-prefix))
      (load (format nil "~a/smartsoft/src/lispInterface/smartInterface.fasl" tcl-prefix))
      (compile-file  (format nil "~a/smartsoft/src/lispInterface/smartInterfaceEvent.lisp" tcl-prefix))
      (load (format nil "~a/smartsoft/src/lispInterface/smartInterfaceEvent.fasl" tcl-prefix)))
    (progn
      (compile-file  (format nil "~a/smartInterface.lisp" lispinterface-prefix))
      (load (format nil "~a/smartInterface.fasl" lispinterface-prefix))
      (compile-file  (format nil "~a/smartInterfaceEvent.lisp" lispinterface-prefix))
      (load (format nil "~a/smartInterfaceEvent.fasl" lispinterface-prefix))))

  ;; execute pre-component-startup-function
  (if pre-component-startup-function-supplied-p 
    (if pre-component-startup-function-param-supplied-p 
      (funcall pre-component-startup-function pre-component-startup-function-param)
      (funcall pre-component-startup-function)))
  
  ;; initialize the c++ component (init, connect, set alive)
  (format t "Starting C++ part of component...~%")
 ; (initialize (format nil "~{~A~^ ~}" sb-ext:*posix-argv*))
  (cffi:with-foreign-strings ((args_str (format nil "~{~A~^ ~}" sb-ext:*posix-argv*)))
    (initialize args_str))

  (compile-file (format nil "~asmartTCL/tcl-basic.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-basic.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/tcl-components.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-components.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/tcl-library.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-library.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/tcl-sync.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-sync.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/tcl-agenda.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-agenda.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/tcl-functions.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-functions.fasl" tcl-prefix))

  (compile-file (format nil "~asmartTCL/decode-msg.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/decode-msg.fasl" tcl-prefix))


  (format t "~%~%")
  (format t " ------------------------------- ~%")
  (format t " compiled and loaded succesfully ~%")
  (format t " SmartTCL ready to use           ~%")
  (format t " ------------------------------- ~%~%"))

;; load ONLY - WIHTOUT PRECOMPILE
(defun load-smarttcl ( &optional (tcl-prefix "") (lispinterface-prefix "") (pre-component-startup-function nil pre-component-startup-function-supplied-p) 
                                                                           (pre-component-startup-function-param nil pre-component-startup-function-param-supplied-p) )
  ;; load smartSimpleKB
  (load (format nil "~asmartTCL/smartSimpleKB/smartSimpleKB.lisp" tcl-prefix))
  ;; Robot Memory
  (defvar *MEMORY* (make-instance 'kb-main-class))

  (load (format nil "~asmartTCL/defs.lisp" tcl-prefix))

  (setf *LISP-INTERFACE-PREFIX* lispinterface-prefix)
  (if (equal lispinterface-prefix "")
    (load "../SmartLispServer/src/lispInterface/smartInterface.lisp")
    (load (format nil "~a/smartInterface.lisp" lispinterface-prefix)))

  ;; execute pre-component-startup-function
  (if pre-component-startup-function-supplied-p 
    (if pre-component-startup-function-param-supplied-p 
      (funcall pre-component-startup-function pre-component-startup-function-param)
      (funcall pre-component-startup-function)))
  
  ;; initialize the c++ component (init, connect, set alive)
  (format t "Starting C++ part of component...~%")
  (initialize (format nil "~{~A~^ ~}" sb-ext:*posix-argv*))

  (load (format nil "~asmartTCL/tcl-basic.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-components.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-library.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-agenda.lisp" tcl-prefix))
  (load (format nil "~asmartTCL/tcl-functions.lisp" tcl-prefix))

  (format t "~%~%")
  (format t " --------------------- ~%")
  (format t " SmartTCL ready to use ~%")
  (format t " --------------------- ~%~%"))

