;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011 Andreas Steck
;;
;;		steck@hs-ulm.de
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

(defvar *CURRENT-INSTANCE* nil)
(defvar *EVT* nil)
(defvar *SMARTSOFT* nil)
(defvar *MSG* nil)
(defvar *ACTIVATED-EVENT-LIST* nil)
(defvar *PENDING-EVENT-LIST* nil)
(defvar *ADAPTATION* nil)
(defvar *COMPONENT-LIFECYCLE-EVENT-LIST* nil)
(defvar *RULE-CHILD-TCB* nil)

(defvar *DEDBUG-CI* nil)

;;(defgeneric show (instance)) ; is already defined in smartSimpleKB

;; SmartTCL
(defgeneric update-slot (instance slot var value))
(defgeneric remove-var-from-slot (instance slot var))
(defgeneric clear-vars (instance slot))
(defgeneric bind-var (instance var value))
(defgeneric clear-tcb-stack (instance))
(defgeneric push-tcb (instance tcb))
(defgeneric push-back-tcb (instance tcb))
(defgeneric push-plan (instance tcb-list))
(defgeneric push-back-plan (instance tcb-list))
(defgeneric pop-tcb (instance))
(defgeneric activate-event (instance evt-name evt-hndlr server service mode param))
(defgeneric activate-component-life-cycle-event (evt-name evt-hndlr server service))
;(defgeneric destroy-event (instance evt)) ;;DOTO CHECK IF THIS ONE IS NEEDED!
(defgeneric select-current-tcb (instance))
(defgeneric execute-action (instance action))
(defgeneric execute-rule (instance tcb))
(defgeneric check-for-any-rule (instance tcb))
(defgeneric update-env-apply-rule (instance tcb))
(defgeneric execute-one-step (instance))  
(defgeneric abort-tcb (instance))
(defgeneric abort-tcb-all (instance))  
(defgeneric interrupt-tcb (instance))

;; SYNC VAR
(defgeneric reset-var (instance))
(defgeneric set-value (instance value))
(defgeneric set-wait (instance tcb-instance))

(defmethod read-sync-variable (instance name))
(defmethod reset-sync-variable (instance name))
(defmethod write-sync-variable (instance name value))
(defmethod wait-for-sync-variable (instance evt-name  var-name evt-hndlr))
(defmethod destroy-event-sync-variable (instance event))
