;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2018 Matthias Lutz
;;
;;		lutz@hs-ulm.de
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
;;; TCL SYNC VAR class which provides several standardized methods
;;;


(defclass TCL-SYNC-VARIABLE ()
  ( (id               :accessor var-id               :initform nil)    ;; id to ref the object
    (name             :accessor var-name             :initform nil)    ;; name to get the idea of the meaning 
    (value            :accessor var-value            :initform nil)    ;; the core value
    (var-wait-called  :accessor var-wait-called      :initform nil)    ;; T in case a wait to the var has been called
    (var-waiting-instances :accessor var-waiting-instances      :initform   nil)
    (waiting-instance-counter :accessor var-waiting-instance-counter :initform 0)
    (var-set          :accessor var-set              :initform nil)    ;; T in case the var is set
    (var-count        :accessor var-count              :initform 0)))



;;; constructor <instance> <id> <name>
;;;    example     : ...
;;;    returns     : an instance of a TCL-SYNC-VARIABLE
(defmethod initialize-instance :after ((instance tcl-sync-variable) &key name)
  (setf (var-id instance) (sb-kernel:get-lisp-obj-address instance))
  (setf (var-name instance) name)
  (format t "Instantiate SYNC VAR id: ~x name: ~s ~%" (var-id instance) name))

(defmethod reset-var ((instance tcl-sync-variable))
 (setf (var-value instance) nil)
 (setf (var-wait-called instance) nil)
 (setf (var-waiting-instances instance) nil)
 (setf (var-waiting-instance-counter instance) 0)
 (setf (var-set instance) nil)
 (setf (var-count instance) 0)
 T)

(defmethod set-value ((instance tcl-sync-variable) value)
  (setf (var-value instance) value)
  (setf (var-set instance) T)
  (incf (var-count instance) 1))

(defmethod set-wait ((instance tcl-sync-variable) tcb-instance)
  (setf (var-wait-called  instance) T)
  (incf (var-waiting-instance-counter instance) 1)
  (push `(,(var-waiting-instance-counter instance) . ,tcb-instance) (var-waiting-instances instance))
  (var-waiting-instance-counter instance))

;;; print method
(defmethod print-object ((instance tcl-sync-variable) out)
  (print-unreadable-object (instance out :type t :identity t)
    (format out "~x ~s var-count:~s" (var-id instance) (var-name instance) (var-count instance) )))
