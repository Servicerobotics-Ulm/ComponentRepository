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


;(defun load-module (wildcard)
;    (dolist (file (directory wildcard)) (load file)))

;(let ((quicklisp-init (merge-pathnames "quicklisp/setup.lisp"
;                                       (user-homedir-pathname))))
;  (when (probe-file quicklisp-init)
;    (load quicklisp-init)))
;
;(ql:quickload "cl-elasticsearch")



(defun load-module (directory)
    (dolist (file (directory (format nil "~a/smartModule*.lisp" directory))) 
      (format t "Load Module file: ~a ...~%" file)
      (load file)))


;; THIS WILL AVOID A WARINING FOR A UNKOWN FUNCTION - initialize, from smartInterface.lisp
(declaim (ftype (function (T) (VALUES (ALIEN INT) &OPTIONAL)) initialize))

(defvar *LISP-INTERFACE-PREFIX* nil)

;; compile and load
(defun compile-and-load-smartjobdispatcher ( &optional (prefix "") (lispinterface-prefix ""))

  (compile-file (format nil "~asmartSimpleKB.lisp" prefix))
  (load (format nil "~asmartSimpleKB.fasl" prefix))

  ;; Robot Memory
  (defvar *MEMORY* (make-instance 'kb-main-class))

  (setf *LISP-INTERFACE-PREFIX* lispinterface-prefix)
  (compile-file (format nil "~a../lispInterface/smartInterface.lisp" prefix))
  (load (format nil "~a../lispInterface/smartInterface.fasl" prefix))

  (compile-file (format nil "~asplit-sequence.lisp" prefix))
  (load (format nil "~asplit-sequence.fasl" prefix))

  (compile-file (format nil "~amath.lisp" prefix))
  (load (format nil "~amath.fasl" prefix))

  (compile-file (format nil "~afunctions.lisp" prefix))
  (load (format nil "~afunctions.fasl" prefix))

  (compile-file (format nil "~ametricFF-jobdispatch.lisp" prefix))
  (load (format nil "~ametricFF-jobdispatch.fasl" prefix))

  (compile-file (format nil "~ametricFF-park-robots.lisp" prefix))
  (load (format nil "~ametricFF-park-robots.fasl" prefix))

  ;(compile-file (format nil "~asmartJobDispatcher.lisp" prefix))
  ;(load (format nil "~asmartJobDispatcher.fasl" prefix))
  
  (compile-file (format nil "~azafh/smartJobDispatcher.lisp" prefix))
  (load (format nil "~azafh/smartJobDispatcher.fasl" prefix))

 ; (compile-file (format nil "~aelastic.lisp" prefix))
 ; (load (format nil "~aelastic.fasl" prefix))

  ;; initialize the c++ component (init, connect, set alive)
  (format t "Starting C++ part of component...~%")

  ;(initialize (format nil "~{~A~^ ~}" sb-ext:*posix-argv*))
  (cffi:with-foreign-strings ((args_str (format nil "~{~A~^ ~}" sb-ext:*posix-argv*)))
    (initialize args_str))


  (format t "~%~%")
  (format t " ------------------------------- ~%")
  (format t " compiled and loaded succesfully ~%")
  (format t " SmartJobDispatcher ready to use ~%")
  (format t " ------------------------------- ~%~%"))


