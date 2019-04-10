;;--------------------------------------------------------------------------
;;
;;  Copyright (C)  2014-2016 Matthias Lutz
;;
;;		lutz@hs-ulm.de
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

;; compile and load
;(defun compile-and-load-simplekb()
(defun load-smartsimplekb ( &optional (prefix ""))

  (compile-file (format nil "~a/smartSimpleKB_lisp/defs.lisp" prefix))
  (load (format nil "~a/smartSimpleKB_lisp/defs.fasl" prefix))

  (compile-file (format nil "~a/smartSimpleKB_lisp/lispInterface/smartInterface.lisp" prefix))
  (load (format nil "~a/smartSimpleKB_lisp/lispInterface/smartInterface.fasl" prefix))

  ;; load smartSimpleKB
  ;(compile-file "src/lisp/smartSimpleKB.lisp")
  (load (format nil "~a/smartSimpleKB_lisp/smartSimpleKB.lisp" prefix))

  ;; Memory
  (defvar *MEMORY* (make-instance 'kb-main-class))
  (defvar *MEMORY-LOCK* (SB-THREAD:make-mutex :name "MAIN LOCK OF KB INSTANCE"))

  (format t "~%~%")
  (format t " ------------------------------- ~%")
  (format t " compiled and loaded succesfully ~%")
  (format t " SmartSimpleKB ready to use      ~%")
  (format t " ------------------------------- ~%~%"))


;; load ONLY
;(defun load-smarttcl ( &optional (prefix ""))
;  ;; load smartSimpleKB
;  (load (format nil "~asmartSimpleKB/smartSimpleKB.fasl" prefix))
;  ;; Robot Memory
;  (defvar *MEMORY* (make-instance 'kb-main-class))
;
;  (load (format nil "~asmartTCL/defs.fasl" prefix))
;  (if (equal prefix "")
;    (load "../SmartLispServer/src/lispInterface/smartInterface.fasl")
;    (load (format nil "~asmartTCL/smartInterface.fasl" prefix)))
;  (load (format nil "~asmartTCL/tcl-basic.fasl" prefix))
;  (load (format nil "~asmartTCL/tcl-components.fasl" prefix))
;  (load (format nil "~asmartTCL/tcl-library.fasl" prefix))
;  (load (format nil "~asmartTCL/tcl-agenda.fasl" prefix))
;  (load (format nil "~asmartTCL/tcl-functions.fasl" prefix))
;
;  (format t "~%~%")
;  (format t " --------------------- ~%")
;  (format t " SmartTCL ready to use ~%")
;  (format t " --------------------- ~%~%"))

