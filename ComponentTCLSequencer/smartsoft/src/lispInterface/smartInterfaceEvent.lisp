;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	1997-2000 Christian Schlegel
;;                      2009/2010 Andreas Steck
;;                      2013-2019 Matthias Lutz
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

;;
;; USER INTERFACE - EVENT CLASS
;;
;;   class event
;;     slots
;;       module          : identifier of the module where the event belongs
;;       module-instance :
;;       ci              :
;;       ci-inst         :
;;       service         : name of the event service
;;       parameter       : parameter for event activation, first parameter is always event mode
;;       mode            : 
;;       state           :
;;       id              :
;;       result          : not yet processed messages with newest message in front of the list
;;
;;     show
;;       description : print all relevant details of an event instance
;;       parameter   : <instance of event>
;;       result      :
;;       remarks     :
;;       example     :
;;
;;     activate
;;       description : activate an already generated event
;;       parameter   : <instance of event> (<parameter of the event>)
;;       result      : (ok ()) or (error (<...>))
;;       remarks     : the event mode has to be specified at event generation, not at event activation
;;       example     : (activate *EVENT1* (100))
;;
;;     deactivate
;;       description : deactivate this event instance
;;       parameter   : <instance of event>
;;       result      : (ok ()) or (error (<...>))
;;       remarks     :
;;       example     : (deactivate *EVENT1*)
;;
;;     check
;;       description : check whether this event has pending messages
;;       parameter   : <instance of event>
;;       result      : T, NIL
;;       remarks     :
;;       example     : (check *EVENT1*)
;;
;;     getmessage
;;       description : get the next event message if one is available and remove it from the list
;;       parameter   : <instance of event>
;;       result      : event result or NIL
;;       remarks     :
;;       example     : (getmessage *EVENT1*)


(defgeneric activate (instance parameter))
(defgeneric deactivate (instance))
(defgeneric check (instance))
(defgeneric getmessage (instance))
(defgeneric show (instance))



(defclass event ()
  ((module    :accessor event-module    :initform nil :initarg :module)
   (module-instance    :accessor event-module-instance    :initform nil :initarg :module-instance)
   (ci        :accessor event-ci :initform nil :initarg :ci)
   (ci-inst   :accessor event-ci-inst :initform nil :initarg :ci-inst)
   (service   :accessor event-service   :initform nil :initarg :service)
   (parameter :accessor event-parameter :initform nil :initarg :parameter)
   (mode      :accessor event-mode      :initform nil :initarg :mode) 
   (state     :accessor event-state     :initform 'new)
   (id        :accessor event-id        :initform nil)
   (result    :accessor event-result    :initform nil)))

(defmethod show ((instance event))
  (format t "Current setting of event instance ~s~%" instance)
  (format t "module     : ~s~%" (event-module instance))
  (format t "module-inst: ~s~%" (event-module-instance instance))
  (format t "ci         : ~s~%" (event-ci instance))
  (format t "ci-inst    : ~s~%" (event-ci-inst instance))
  (format t "service    : ~s~%" (event-service instance))
  (format t "parameter  : ~s~%" (event-parameter instance))
  (format t "id         : ~s~%" (event-id instance))
  (format t "state      : ~s~%" (event-state instance))
  (format t "mode       : ~s~%" (event-mode instance))
  (format t "result     : ~s~%" (event-result instance)))

(defmethod activate ((instance event) parameter)
  (cond ((equal (event-state instance) 'new)
          (setf (event-parameter instance) parameter)
          (setf (event-mode instance) (first parameter))
          (if *DEBUG-LI* (show instance)) ;;debug
          (let ((tmp (interface (event-module instance) (event-module-instance instance) (event-ci-inst instance) (event-service instance) (format nil "~a-activate" (event-service instance)) parameter)))
               (cond ((equal (first tmp) 'ok)
                       (setf (event-id instance) (first (second tmp)))
                       (setf (event-state instance) 'activated)
                       (list 'ok '()))
                     (T tmp))))
        (T (list 'error '(wrong event state for activation)))))

(defmethod deactivate ((instance event))
  (if *DEBUG-LI* (show instance)) ;;debug
  (cond ((equal (event-state instance) 'activated)
          (let ((tmp (interface (event-module instance) (event-module-instance instance) (event-ci-inst instance) (event-service instance) (format nil "~a-deactivate" (event-service instance)) (list (event-id instance)))))
               (cond ((equal (first tmp) 'ok)
                       (setf (event-state instance) 'deactivated)
                       (list 'ok '()))
                     (T tmp))))
        (T (list 'error '(wrong event state for deactivation)))))

(defmethod check ((instance event))
  (not (null (event-result instance))) )

(defmethod getmessage ((instance event))
  (let ( (tmp (event-result instance)) )
    (setf (event-result instance) (butlast tmp))
    (car (last tmp)) ))

