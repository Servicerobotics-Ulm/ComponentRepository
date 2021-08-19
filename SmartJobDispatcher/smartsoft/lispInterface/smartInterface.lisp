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
;  Author: Christian Schlegel, Andreas Steck, Matthias Lutz
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

;;;
;;   cd ~/SOFTWARE/smartsoft/src/myComps/smartLispServer/lispInterface/
;;   sbcl
;;   (load "smartInterface.lisp")
;;
;;   (communication *SMARTSOFT* (list 'tts 'command 'say (list "Hello World!")))


(format t "
~%
----------------------------------------------------------------
LISP/C++ Interface
Christian Schlegel, Andreas Steck, Matthias Lutz
ZAFH Servicerobotik Ulm, Germany
1997-2000, 2009, 2010, 2014
----------------------------------------------------------------
~%")

(defvar *CLASS-MAP-L* nil)
(defvar *MODULE-MAP-L* nil)
(defvar *COMMOBJ-MAP-L* nil)

(defvar *SMARTSOFT* nil)

;; LISP interface
(defgeneric generate-event (instance parameter))
(defgeneric destroy-event (instance parameter))
(defgeneric get-event (instance identifier))
(defgeneric process-event-results (instance))
(defgeneric check-events (instance eventlist))
(defgeneric check-events-wait (instance eventlist))
(defgeneric communication (instance activity))
(defgeneric activate (instance parameter))
(defgeneric deactivate (instance))
(defgeneric check (instance))
(defgeneric getmessage (instance))

;;(setq *SOFILE* "~/scenarios/asteck-masterthesis/smartLispServer.so")
;;(setq *SOFILE* "~/SOFTWARE/smartsoft/bin/smartLispServer.so")
;;(setq *SOFILE* "./smartLispServer.so")

;;
;; mapping functions and low level interface
;;
;;   map-module-to-code
;;     description : map module name to integer module id
;;     parameter   : <module name>
;;     result      : <module id>
;;     remarks     : --- INTERNAL USE ONLY ---
;;     example     :
;;
;;   map-class-to-code
;;     description :
;;     parameter   : <module name> <service name> <method name>
;;     result      : <C interface function id>
;;     remarks     : --- INTERNAL USE ONLY ---
;;     example     :
;;
;;   sendCommand
;;     description : sends a command "as is" to the low level interface
;;     parameter   : <module name> <service name> <method name> <parameter>
;;     result      : <string which is input for read from string>
;;     remarks     : --- INTERNAL USE ONLY ---
;;     example     :
;;
;;
;;   interface
;;     description : interact with external SmartSoft modules
;;     parameter   : <module> <service> <method> <parameter>
;;     result      : returns original string from low level interface
;;                   (ok ()) or (error (<error message>))
;;     remarks     : does not administrate internal event structures
;;                   --- INTERNAL USE ONLY ---
;;     example     : (interface 'cdl 'stuckevent 'activate '(continuous 100))

(defun map-module-to-code (mod)
  (dolist (l *MODULE-MAP-L*) 
    (if (equal (first l) mod) 
      (return (second l)))))

(defun map-class-to-code (mod svc mth)
  (dolist (l *CLASS-MAP-L*) 
    (if (equal (first l) mod) 
      (return (dolist (lf (second l)) 
        (if (equal (first lf) svc)
          (return (dolist (lff (second lf))
            (if (equal (first lff) mth)
              (return (second lff)))))))))))

(defun apply-parameter-substitution (mod param)
  (cond 
    ;; each param could be a commit, locally defined
    ((string-equal param "COMMIT")
      param)
    (T
      (dolist (l *COMMOBJ-MAP-L*) 
        (if (equal (first l) mod) 
         (return-from apply-parameter-substitution (concatenate-objects-to-symbol (second l) param ))))
      ;;in case not match is found use the parameter as it is
      param)))

(defun concatenate-objects-to-symbol (&rest objects)
  (intern (apply #'concatenate 'string (mapcar #'princ-to-string objects))))


(defun sendCommand (mod svc mth prm)
  (let ((tmp nil))
       (push prm tmp)
       (push (map-class-to-code mod svc mth) tmp)
       (push (map-module-to-code mod) tmp)
       (cond
         ((or (null (first tmp)) (null (second tmp)))
           (format t "ERROR mapping module or function!~%")
           "(error (unkown module or function))")
         (T
           ;(format t "sendCommand: ~s~%" tmp)
           (let ((ptr nil)
                 (result nil))
             (cffi:with-foreign-strings ((tmp_str (write-to-string tmp)))
               (setf ptr (command tmp_str)))
             (setf result (cffi:foreign-string-to-lisp ptr))
             (cffi:foreign-free ptr)
             result)))))

(defun interface (mod svc mth prm)
  (read-from-string (sendCommand mod svc mth prm)))

(defun wait-for-message ()
 (let ((ptr (getjobmsg))
       (result nil))
   (setf result (cffi:foreign-string-to-lisp ptr))
   (cffi:foreign-free ptr)
   result))

;(defun send-job (job robot)
;  (sendjobmsg (format nil "~a" job) (format nil "~a" robot )))

(defun send-job (job robot)
   (format t "send-job job:~a robot:~a~%" job robot)
  (cffi:with-foreign-strings ((job_str (write-to-string job)) (robot_str (write-to-string robot)))
    (sendjobmsg job_str robot_str)))
;;
;; USER INTERFACE
;;
;;   class smartsoft
;;     slots
;;       instantiated-events 
;;       result
;;
;;     generate-event
;;       description : generate a new event instance
;;       parameter   : <instance of smartsoft> (<module> <service> (<mode> <further parameter>))
;;       result      : <new event instance>
;;       result slot : (does not fill result slot)
;;       remarks     :
;;       example     : (generate-event *SMARTSOFT* '(cdl stuckevent (continuous)))
;;
;;     destroy-event
;;       description : destroy an event instance
;;       parameter   : <instance of smartsoft> (<instance of event>)
;;       result      :
;;       result slot : (does not fill result slot)
;;       remarks     :
;;       example     : (destroy-event *SMARTSOFT* (list *event1* ))
;;
;;     get-event
;;       description : returns event instance which belongs to identifier
;;       parameter   : <instance of smartsoft> (<module> <service> <event id>)
;;       result      : <instance of matching event>, NIL
;;       result slot : (does not fill result slot)
;;       remarks     :
;;       example     : (get-event *SMARTSOFT* '(cdl stuckevent 3))
;;
;;     process-event-results
;;       description : get event messages and distribute them to the appropriate event instances
;;       parameter   : <instance of smartsoft>
;;       result      : T or NIL
;;       result slot : NIL, (unknown error), (unknown method number), (too many event messages)
;;       remarks     : 
;;       example     : (process-event-result *SMARTSOFT*)
;;
;;     check-events-wait
;;       description : waits until at least one pending message is available within the specified events
;;       parameter   : <instance of smartsoft> <list of event instances>
;;       result      : <list of events which have pending events>
;;       result slot : (does not fill result slot)
;;       remarks     : calls process-events-results itself !
;;       example     : (wait-for *SMARTSOFT* (list e1 e2 e3))
;;
;;     communication
;;
;;       This is the main user interface function
;; 
;;       description : handle all commands to other modules including event administration
;;       parameter   : <instance of smartsoft> (<module> <service> <method> (<further parameter>))
;;       result      : generate event : <instance of generated event>
;;                     otherwise      : T or NIL with result slot set
;;       result slot : NIL and all possible error messages from low level interface
;;                     and all possible queried results from low level interface
;;       remarks     : 
;;       example     : (communication *SMARTSOFT* '(special event generate   '(cdl stuckevent continuous)))
;;                     (communication *SMARTSOFT* '(special event activate   (list *event1* 100)))
;;                     (communication *SMARTSOFT* '(special event deactivate (list *event1* )))
;;                     (communication *SMARTSOFT* '(special event destroy    (list *event1* )))
;;                     (communication *SMARTSOFT* '(cdl command lookuptable '(default)))

(defclass smartsoft ()
  ((instantiated-events :accessor smartsoft-instantiated-events :initform nil)
   (result              :accessor smartsoft-result              :initform nil)))

(defmethod generate-event ((instance smartsoft) parameter)
  (let* ((mod (first  parameter))
         (svc (second parameter))
         (prm (third  parameter))
         (mde (first  prm))
         (tmp (make-instance 'event :module mod :service svc :parameter prm :mode mde)))
        (push tmp (smartsoft-instantiated-events instance))
        tmp))

(defmethod destroy-event ((instance smartsoft) parameter)
  (let ((event-inst (first parameter)))
       (cond ((equal (event-state event-inst) 'new)
                (setf (smartsoft-instantiated-events instance) (delete event-inst (smartsoft-instantiated-events instance)))
                (setf (event-state event-inst) 'destroyed)
                (list 'ok '()))
             ((equal (event-state event-inst) 'deactivated)
                (setf (smartsoft-instantiated-events instance) (delete event-inst (smartsoft-instantiated-events instance)))
                (setf (event-state event-inst) 'destroyed)
                (list 'ok '()))
             ((equal (event-state event-inst) 'activated)
                (list 'error '(cannot destroy activated event)))
             ((equal (event-state event-inst) 'destroyed)
                (list 'error '(cannot destroy already destroyed event)))
             (T (list 'error '(cannot destroy event with unknown event state))))))

(defmethod get-event ((instance smartsoft) identifier)
  (let ( (mod (first  identifier))
         (svc (second identifier))
         (eid (third  identifier))
         (tmp nil)
       )
       (dolist (l (smartsoft-instantiated-events instance))
         (cond ( (and (equal mod (event-module l)) 
                      (equal svc (event-service l)) 
                      (equal eid (event-id l))
                 )
                 (setq tmp l)
               )
         )
       )
       tmp
  )
)
                
(defmethod process-event-results ((instance smartsoft))
  (let* ( (resultList (interface 'fetchEvents 'eventResultList 'get '()))
          (status (first resultList))
          (messages (second resultList))
        )
        (cond 
               (( equal status 'ok)
                 ; returned messages are ok
                 (setf (smartsoft-result instance) nil)
                 (dolist (l messages)
                   (if (not (equal l '()))
                     ; returns empty list if no event messages available
                     (let* ( (eventIdentifier (first l))
                             (eventResult (second l))
                             (eventptr (get-event instance eventIdentifier))
                           )
                           ;(format t "eventIdentifier ~s~%" eventIdentifier)
                           ;(format t "eventResult ~s~%" eventResult)
                           ;(format t "eventptr ~s~%" eventptr)
                           (cond
                             ((not (equal eventptr nil))
                               (push eventResult (event-result eventptr))))))))
               ((equal status 'error)
                 (setf (smartsoft-result instance) messages)
                 (format t "~% an error occured: ~s~%" messages)
                 ;(pprint messages)
                 NIL)
               (T (setf (smartsoft-result instance) messages)
                 (format t "~% unknown error: ~s~%" messages)
                 ;(pprint messages)
                 NIL))))

(defmethod check-events ((instance smartsoft) eventlist)
  (process-event-results instance)
  (loop for event in eventlist when (check event) collect event))


(defmethod check-events-wait ((instance smartsoft) eventlist)
  (let ((result nil))
    (loop
      (process-event-results instance)
      (setf result (loop for event in eventlist when (check event) collect event))
      (cond ((null result)
              (delay 100000))
            (T (return result))))))


(defmethod communication ((instance smartsoft) activity)
  (let ( (result nil)
         (mod (first  activity))
         (svc (second activity))
         (mth (third  activity))
         (prm (fourth activity)))
       ;; special - event - generate
       (cond ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'generate))
               (let ( (event-mod (first  prm))
                      (event-svc (second prm))
                      (event-mde (third  prm))
                    )
                    ;; generate event
                    (setq result (generate-event instance (list event-mod event-svc (list event-mde))))
		   ;; (show result) ;;DEBUG
               )
             )
             ;; special - event - activate
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'activate)
               )
               (let ( (event-inst (first prm))
                      (event-prm  (rest  prm))
                    )
                    ;; activate event
                    (setq result (activate event-inst (list (event-mode event-inst) event-prm)))
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T
                          )
                          ;; result error (known error)
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% activate event an error occured: ~s~%" activity)
                              NIL
                          )
                          ;; result error (unknown error)
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% activate event unknown error: ~s~%" activity)
                              NIL
             ))))
             ;; special - event - destroy
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'destroy)
               )
               (let ( (event-inst (first prm))
                    )
                    ;; destroy event     
                    (setq result (destroy-event instance (list event-inst)))
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T
                          )
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% destroy event an error occured: ~s~%" activity)
                              NIL
                          )
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% destroy event unknown error ~s~%" activity)
                              NIL
             ))))
             ;; special - event - deactivate
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'deactivate)
               )
               (let ( (event-inst (first prm))
                    )
                    ;; deactivate event
                    (setq result (deactivate event-inst))
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T
                          )
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% deactivate event an error occured: ~s -- result: ~s~%" activity result)
                              NIL
                          )
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% deactivate event unknown error: ~s -- result: ~s~%" activity result)
                              NIL
             ))))
             ;; T
             ( T 
                 ;; interface
                 (setq result (interface mod svc mth prm))
                 ;; result ok
                 (cond ( (equal (first result) 'ok)
                           (setf (smartsoft-result instance) (second result))
                           T
                       )
                       ( (equal (first result) 'error)
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: an error occured: ~s -- result: ~s ~%" activity result)
                           NIL
                       )
                       ( T 
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: unknown error: ~s~%" activity)
                           NIL
                       )
)))))

;;
;; USER INTERFACE
;;
;;   class event
;;     slots
;;       module      : identifier of the module where the event belongs to (smartModule<>.lisp)
;;       service     : name of the event (smartModule<>.lisp)
;;       parameter   : parameter for event activation, first parameter is always event mode
;;       mode        : 
;;       state       :
;;       id          :
;;       result      : not yet processed messages with newest message in front of the list
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

(defclass event ()
  ((module    :accessor event-module    :initform nil :initarg :module)
   (service   :accessor event-service   :initform nil :initarg :service)
   (parameter :accessor event-parameter :initform nil :initarg :parameter)
   (mode      :accessor event-mode      :initform nil :initarg :mode) 
   (state     :accessor event-state     :initform 'new)
   (id        :accessor event-id        :initform nil)
   (result    :accessor event-result    :initform nil)))

(defmethod show ((instance event))
  (format t "Current setting of event instance ~s~%" instance)
  (format t "module    : ~s~%" (event-module instance))
  (format t "service   : ~s~%" (event-service instance))
  (format t "parameter : ~s~%" (event-parameter instance))
  (format t "id        : ~s~%" (event-id instance))
  (format t "state     : ~s~%" (event-state instance))
  (format t "mode      : ~s~%" (event-mode instance))
  (format t "result    : ~s~%" (event-result instance)))
;;  (return-from show instance))

(defmethod activate ((instance event) parameter)
  (cond ((equal (event-state instance) 'new)
          (setf (event-parameter instance) parameter)
          (setf (event-mode instance) (first parameter))
          (let ((tmp (interface (event-module instance) (event-service instance) 'activate parameter)))
               (cond ((equal (first tmp) 'ok)
                       (setf (event-id instance) (first (second tmp)))
                       (setf (event-state instance) 'activated)
                       (list 'ok '()))
                     (T tmp))))
        (T (list 'error '(wrong event state for activation)))))

(defmethod deactivate ((instance event))
  (cond ((equal (event-state instance) 'activated)
          (let ((tmp (interface (event-module instance) (event-service instance) 'deactivate (list (event-id instance)))))
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
  

;;
;;
;; system functions to handle the SmartSoft interface
;;
;;


;; alisp
;;(ff:defforeign 'command :arguments '(integer) :return-type :integer)
;;(ff:defforeign 'initialize :return-type :void)
;;(ff:defforeign 'delay :arguments '(integer) :return-type :void)
;;(load *SOFILE*) ;; load shared object in alisp

;;(load-shared-object *SOFILE*)

(cond 
  ((probe-file (format nil "~a/bin/libSmartJobDispatcher.so" (sb-ext:posix-getenv "SMART_ROOT_ACE")))
   (format t "LOADING libSmartJobDispatcher.so from env SMART_ROOT_ACE context!~%~%")
   (cffi:load-foreign-library (format nil "~a/bin/libSmartJobDispatcher.so" (sb-ext:posix-getenv "SMART_ROOT_ACE"))))
  ((probe-file "/usr/bin/smartSimpleKB/libSmartJobDispatcher.so") 
   (format t "LOADING INSTALLED /usr/bin/smartJobDispatcher/libSmartJobDispatcher.so~%~%")
   (cffi:load-foreign-library "/usr/bin/smartJobDispacher/libSmartJobDispatcher.so.so"))
  ((probe-file "libSmartJobDispatcher.so") 
   (format t "LOADING libSmartJobDispatcher.so from SYSTEM context!~%~%")
   (cffi:load-foreign-library "libSmartJobDispatcher.so"))
  ((probe-file (format nil "~a/bin/SmartJobDispatcher.dll" (sb-ext:posix-getenv "SMART_ROOT_ACE")))
   (format t "LOADING SmartJobDispatcher.dll from env SMART_ROOT_ACE context!~%~%")
   (cffi:load-foreign-library (format nil "~a/bin/SmartJobDispatcher.dll" (sb-ext:posix-getenv "SMART_ROOT_ACE"))))
   
  (T (format t "ERROR loading libSmartJobDispatcher.so file not found!~%~%")(exit)))


;(SB-ALIEN:define-alien-routine delay void (x int))
;(SB-ALIEN:define-alien-routine command (c-string) (inString (c-string)))
;(SB-ALIEN:define-alien-routine getjobmsg (c-string))  
;(SB-ALIEN:define-alien-routine sendjobmsg void (jobmsg (c-string))(robot (c-string)))  
;(SB-ALIEN:define-alien-routine initialize int (paramFile (c-string)))
;(SB-ALIEN:define-alien-routine waitoncomptasktocomplete void)


(cffi:defcfun "delay" :void (x :int))
(cffi:defcfun "command" :pointer (inString :string))
(cffi:defcfun "getjobmsg" :pointer)
(cffi:defcfun "sendjobmsg" :void (jobmsg :string) (robot :string))
(cffi:defcfun "initialize" :int (paramFile :string))
(cffi:defcfun "waitoncomptasktocomplete" :void)


(setq *SMARTSOFT* (make-instance 'smartsoft))

(cond 
  ((probe-file (format nil "~a" *LISP-INTERFACE-PREFIX*))
   (format t "Loading lisp interface modules form ~a~%" *LISP-INTERFACE-PREFIX*)
   (load-module (format nil "~a" *LISP-INTERFACE-PREFIX*)))
  (T (format t "ERROR loading interface modules!~%~%")(exit)))

;;(initialize (format nil "~{~A~^ ~}" sb-ext:*posix-argv*))
