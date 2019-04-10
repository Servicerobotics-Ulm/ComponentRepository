;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	1997-2000 Christian Schlegel
;;					2009/2010 Andreas Steck
;;					2013-2016 Matthias Lutz
;;
;;		lutz@hs-ulm.de
;;		schlegel@hs-ulm.de
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

(format t "
~%
----------------------------------------------------------------
LISP/C++ Interface
Christian Schlegel, Andreas Steck, Matthias Lutz
ZAFH Servicerobotik Ulm, Germany
1997-2000, 2009, 2010, 2014
----------------------------------------------------------------
~%")

(defun get-query ()
 (let ((ptr (getquery))
       (result nil))
   (setf result (cffi:foreign-string-to-lisp ptr))
   (cffi:foreign-free ptr)
   result))

(defun answer-query (answer id)
  (cffi:with-foreign-strings ((answer_str (format nil "~s" answer)))
    (answerquery  answer_str id)))

(defun get-check-param-event ()
  (let ((ptr (getcheckparamevent))
        (result nil))
    (setf result (cffi:foreign-string-to-lisp ptr))
    (cffi:foreign-free ptr)
    result))
       

(defun answer-check-event-param (answer fireEvent formated-answer)
  (cffi:with-foreign-strings ((formated_answer_str (format nil "~a" formated-answer))
                              (answer_str (format nil "~s" answer)))
    (answercheckeventparam answer_str fireEvent formated_answer_str)))

(defvar *KBCLIB* "libComponentKB.so")

(cond 
  ((probe-file *KBCLIB*) 
   (format t "LOADING ~a from local dir!~%~%" *KBCLIB*)
   (cffi:load-foreign-library *KBCLIB*))
  ((probe-file (format nil "~a/bin/~a" (sb-ext:posix-getenv "SMART_ROOT_ACE") *KBCLIB*))
   (format t "LOADING ~a from env SMART_ROOT_ACE context!~%~%" *KBCLIB*)
   (cffi:load-foreign-library (format nil "~a/bin/~a" (sb-ext:posix-getenv "SMART_ROOT_ACE") *KBCLIB*)))
  ((probe-file (format nil "/opt/smartSoftAce/bin/smartSimpleKB/~a" *KBCLIB*))
   (format t "LOADING INSTALLED /opt/smartSoftAce/bin/smartSimpleKB/~a~%~%" *KBCLIB*)
   (cffi:load-foreign-library (format nil "/opt/smartSoftAce/bin/smartSimpleKB/~a") *KBCLIB*))
  ((probe-file (format nil "~a/bin/ComponentKB.dll" (sb-ext:posix-getenv "SMART_ROOT_ACE")))
   (format t "LOADING Component.dll from env SMART_ROOT_ACE context!~%~%")
   (cffi:load-foreign-library (format nil "~a/bin/Component.dll" (sb-ext:posix-getenv "SMART_ROOT_ACE"))))

  (T (format t "ERROR loading ~a file not found!~%~%" *KBCLIB*)(exit)))


;(SB-ALIEN:define-alien-routine delay void (x int))
;(SB-ALIEN:define-alien-routine command (c-string) (inString (c-string)))
;(SB-ALIEN:define-alien-routine getquery (c-string))  
;(SB-ALIEN:define-alien-routine answerquery void (inString (c-string)) (id int))  
;(SB-ALIEN:define-alien-routine initialize int (paramFile (c-string)))
;;KB event interface
;(SB-ALIEN:define-alien-routine memorychanged void)
;(SB-ALIEN:define-alien-routine getcheckparamevent (c-string))


;(SB-ALIEN:define-alien-routine answercheckeventparam void (inString (c-string)) (fireEvent (boolean)) (formated-answer (c-string)) )
;(SB-ALIEN:define-alien-routine waitoncomptasktocomplete void)
;(SB-ALIEN:define-alien-routine registerslavedentry int (key (c-string)) (value (c-string)))

(cffi:defcfun "getquery" :pointer)
(cffi:defcfun "answerquery" :void (inString :string) (id :int))
(cffi:defcfun "initialize" :int (paramFile :string))
(cffi:defcfun "waitoncomptasktocomplete" :void)
;KB event interface
(cffi:defcfun "memorychanged" :void)
(cffi:defcfun "getcheckparamevent" :pointer)
(cffi:defcfun "answercheckeventparam" :void (inString :string) (fireEvent :boolean) (formated-answer :string))
;KB slaves
(cffi:defcfun "registerslavedentry" :int (key :string) (value :string))

(cffi:with-foreign-strings ((args_str (format nil "~{~A~^ ~}" sb-ext:*posix-argv*)))
  (initialize args_str))
