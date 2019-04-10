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


(format t "
~%
----------------------------------------------------------------
LISP/C++ Interface
Christian Schlegel, Andreas Steck, Matthias Lutz
ZAFH Servicerobotik Ulm, Germany
1997-2000, 2009, 2010, 2014-2019
----------------------------------------------------------------
~%")

(defvar *SYNC-VAR-EVENTS* nil)


;; LISP interface
(defgeneric generate-event (instance parameter))
(defgeneric destroy-event (instance parameter))
(defgeneric get-event (instance identifier))
(defgeneric process-event-results (instance))
(defgeneric check-events (instance eventlist))
(defgeneric check-events-wait (instance eventlist))
(defgeneric communication (instance activity))
;;
;;   sendCommand
;;     description : sends a command "as is" to the low level interface
;;     parameter   : <module name> <service name> <method name> <parameter>
;;     result      : <results string from the C call>
;;     remarks     : --- INTERNAL USE ONLY ---
;;     example     : 
;;
;;   interface
;;     description : interact with external SmartSoft modules
;;     parameter   : <module> <module-inst> <service> <method> <parameter>
;;     result      : returns original string from low level interface
;;                   (ok ()) or (error (<error message>))
;;     remarks     : does not administrate internal event structures
;;                   --- INTERNAL USE ONLY ---
;;     example     : (interface 'cdl 'stuckevent 'activate '(continuous 100))


(defun matching-ci-inst (name element)
  (format t "name:~s element:~s~%" name element)
  (equal-symbol-string (cdr (assoc 'INST-NAME element)) name))


(defun sendCommand (mod-type mod-inst ci-inst-name svc service prm)
  (format t "~%~%SEND COMMAND START~%")
  (format t "mod-type: ~s~%" mod-type)
  (format t "mod-inst: ~s~%" mod-inst)
  (format t "ci-inst-name: ~s~%" ci-inst-name)
  (format t "svc: ~s~%" svc)
  (format t "service: ~s~%" service)
  (format t "prm: ~s~%" prm)

  ;;TODO REMOVE COMP_TYPE
  (let ((comp-type nil)
        (ci-inst nil)
        (ci-type nil)
        (ci-services nil)
        (ci-comp-inst nil)
        (module (query-kb *MEMORY* '(is-a type inst-name) `((is-a coordination-module) (type ,mod-type) (inst-name ,mod-inst)))))

        (format t "comp-interfaces  : ~a ~%" (get-value module 'comp-interfaces))

        (setf ci-inst (find ci-inst-name (get-value module 'comp-interfaces) :test #'matching-ci-inst))

        (cond
          ((null ci-inst)
            (format t "ERROR - No matching ci-inst found!~%")
            "(error no ci-inst)")
          (T
            (setf ci-type  (cdr (assoc 'type  ci-inst)))
            (setf ci-services (cdr (assoc 'services ci-inst)))
            (setf ci-comp-inst (cdr (assoc 'component-inst ci-inst)))

            (format t "type             : ~a ~%" ci-type)
            (format t "inst-name        : ~a ~%" ci-inst-name)
            (format t "services         : ~a ~%" ci-services)
            (format t "cm-inst          : ~a ~%" ci-comp-inst)

            (cffi:with-foreign-strings ((ciType_str (format nil "~a" ci-type))
                                        (ciInstance_str (format nil "~a.~a" mod-inst ci-inst-name))
                                        (compType_str (format nil "~a" comp-type))
                                        (compInstace_str (format nil "~a" ci-comp-inst))
                                        (service_str (format nil "~a" service))
                                        (param_str (format nil "~s" prm)))
  ;;TODO REMOVE COMP_TYPE
              (let ((ptr (command ciType_str ciInstance_str compType_str compInstace_str service_str param_str))
                    (result nil))
                (setf result (cffi:foreign-string-to-lisp ptr))
                (cffi:foreign-free ptr)
              (format t "raw res:~s~%" result)
              result))))))


(defun interface (mod mod-inst comp svc mth prm)
  (read-from-string (sendCommand mod mod-inst comp svc mth prm)))

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
 ; (format t "generate-event ~s~%" parameter)
  (let* ((mod (first  parameter))
         (mod-inst (second  parameter))
         (ci-inst-name (third  parameter))
         (ci-type nil)
         (svc (fourth parameter))
         (prm (fifth  parameter))
         (mde (first  prm))
         (module (query-kb *MEMORY* '(is-a type inst-name) `((is-a coordination-module) (type ,mod) (inst-name ,mod-inst))))
         (tmp nil))
  
         (setf ci-type (cdr (assoc 'type (find ci-inst-name (get-value module 'comp-interfaces) :test #'matching-ci-inst))))

;(format t "mod: ~s~%" mod)
;(format t "mod-inst: ~s~%" mod-inst)
;(format t "ci-type: ~s~%" ci-type)
;(format t "ci-inst-name: ~s~%" ci-inst-name)
;(format t "svc: ~s~%" svc)
;(format t "prm: ~s~%" prm)
;(format t "mde: ~s~%" mde)

        (setf tmp (make-instance 'event :module mod :module-instance mod-inst :ci ci-type :ci-inst ci-inst-name :service svc :parameter prm :mode mde))
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

(defmethod get-event ((instance smartsoft) msg)
  (let* ((ci-type  (cdr (assoc 'COORDINATION-INTERFACES-TYPE msg)))
         (ci-inst-with-prefix  (cdr (assoc 'COORDINATION-INTERFACE-INSTANCE msg)))
         (svc-name (cdr (assoc 'service-name msg)))
         (eid      (cdr (assoc 'id msg)))
         (mod-inst-name (first (split-namespace ci-inst-with-prefix)))
         (ci-inst-name (string (second (split-namespace ci-inst-with-prefix))))
         (matched-event nil))

         (dolist (l (smartsoft-instantiated-events instance))
           (format t "get-event match  mod-inst: ~s | ~s ~% inst: ~s | ~s ~% serv: ~s | ~s  ~% id: ~s | ~s~%" (event-module-instance l) mod-inst-name (event-ci-inst l) ci-inst-name (event-service l) svc-name (event-id l) eid)
           (show l)
           (cond 
             ((and (equal-symbol-string mod-inst-name (event-module-instance l))
                   (equal-symbol-string ci-inst-name (event-ci-inst l)) 
                   (equal-symbol-string svc-name (event-service l)) 
                   (equal eid (event-id l)))
               (setq matched-event l))))
         matched-event))

                
(defmethod process-event-results ((instance smartsoft))
  (let* ( (resultList (interface *FETCHEVENT-MODULE-NAME* (intern (string-upcase *FETCHEVENT-MODULE-NAME*)) *FETCHEVENT-MODULE-NAME* 'eventResultList 'get '()))
          (status (first resultList))
          (messages (second resultList))
          (decoded-event-messages nil))
        (cond 
               (( equal status 'ok)
               
                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                 ;; add sync events
                 ;(format t "~%resultList :~s ~%" resultList)
                 (dolist (event *SYNC-VAR-EVENTS*)
                     (push event messages))
                 ;(format t "messages :~s ~%" messages)
                 (setf *SYNC-VAR-EVENTS* nil)
                 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

                 ; returned messages are ok
                 (setf (smartsoft-result instance) nil)
                 (dolist (msg messages)
                   (if (not (equal msg '()))
                     ; returns empty list if no event messages available
                     (let* ( (eventResult (cdr (assoc 'data msg)))
                             (eventptr (get-event instance msg)))
                           ;(format t "eventMessage ~s~%" msg)
                           ;(format t "eventResult ~s~%" eventResult)
                           ;(format t "eventptr ~s~%" eventptr)
                           (cond
                             ((not (equal eventptr nil))
                               (push eventResult (event-result eventptr)))
                             (T
                               (format t "Not able to Match Event!~%")))))))
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
         (mod-inst (second  activity))
         (comp (third  activity))
         (svc (fourth activity))
         (mth (fifth  activity))
         (prm (sixth activity)))

  (format t "~%==================================~%")
  (format t "==================================~%~%")
  (format t "mod: ~s~%" mod)
  (format t "mod-inst: ~s~%" mod-inst)
  (format t "comp-ci-inst: ~s~%" comp)
  (format t "svc: ~s~%" svc)
  (format t "mth: ~s~%" mth)
  (format t "prm: ~s~%" prm)
       ;; special - event - generate
       (cond ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'generate))
               (let ( (event-mod (first  prm))
                      (event-mod-inst (second  prm))
                      (event-comp (third prm))
                      (event-svc (fourth prm))
                      (event-mde (fifth  prm)))
                    ;; generate event
                    (setq result (generate-event instance (list event-mod event-mod-inst event-comp event-svc (list event-mde))))))

             ;; special - event - activate
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'activate))
               (let ( (event-inst (first prm))
                      (event-prm  (rest  prm)))
                    ;; activate event
                    (setq result (activate event-inst (list (event-mode event-inst) event-prm)))
                    (format t "Event past activate: ~%")
                    (show event-inst)
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T)
                          ;; result error (known error)
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% activate event an error occured: ~s~%" activity)
                              NIL)
                          ;; result error (unknown error)
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% activate event unknown error: ~s~%" activity)
                              NIL))))

             ;; special - event - destroy
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'destroy))
               (let ( (event-inst (first prm)))
                    ;; destroy event     
                    (setq result (destroy-event instance (list event-inst)))
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T)
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% destroy event an error occured: ~s~%" activity)
                              NIL)
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% destroy event unknown error ~s~%" activity)
                              NIL))))
             ;; special - event - deactivate
             ( (and (equal mod 'special)
                    (equal svc 'event)
                    (equal mth 'deactivate))
               (let ( (event-inst (first prm)))
                    ;; deactivate event
                    (setq result (deactivate event-inst))
                    ;; result ok
                    (cond ( (equal (first result) 'ok)
                              (setf (smartsoft-result instance) nil)
                              T)
                          ( (equal (first result) 'error)
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% deactivate event an error occured: ~s -- result: ~s~%" activity result)
                              NIL)
                          ( T 
                              (setf (smartsoft-result instance) (second result))
                              (format t "~% deactivate event unknown error: ~s -- result: ~s~%" activity result)
                              NIL))))

             ;; special - KB!
             ( (and (equal mod 'special)
                    (equal mod-inst *KB-MODULE-NAME*))
               (let* ((kb-module (query-kb *MEMORY* '(is-a type) `((is-a coordination-module) (type ,*KB-MODULE-NAME*))))
                      (kb-module-inst (get-value kb-module 'inst-name))
                      (comp-interfaces (get-value kb-module 'comp-interfaces))
                      ;;take the first ci in the KB mode
                      (kb-ci-inst (cdr (assoc 'INST-NAME (first comp-interfaces)))))

                   (show-modules);;DEBUG

               (setq result (interface *KB-MODULE-NAME* kb-module-inst kb-ci-inst svc mth prm)))
               ;; result ok
               (cond ( (equal (first result) 'ok)
                           (setf (smartsoft-result instance) (second result))
                           T)
                       ( (equal (first result) 'error)
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: an error occured: ~s -- result: ~s ~%" activity result)
                           NIL)
                       ( T 
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: unknown error: ~s~%" activity)
                           NIL)))

             ;; T
             ( T 
                 ;; interface
                 (setq result (interface mod mod-inst comp svc mth prm))
                 ;; result ok
                 (cond ( (equal (first result) 'ok)
                           (setf (smartsoft-result instance) (second result))
                           T)
                       ( (equal (first result) 'error)
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: an error occured: ~s -- result: ~s ~%" activity result)
                           NIL)
                       ( T 
                           (setf (smartsoft-result instance) (second result))
                           (format t "~% communication interface: unknown error: ~s~%" activity)
                           NIL))))))



(defun generate-ci-service-map (type ciInstanceName)
  (format t "generate-ci-service-map - TYPE: ~s CI-INST: ~s~%" type ciInstanceName)
  (let ((ci-list (query-kb-all *MEMORY* '(is-a type inst-name) `((is-a module) (type ,type) (inst-name ,ciInstanceName))))
        (result (make-string-output-stream)))
    (format t "ci-list: ~a~%" ci-list)
    ;;there should be only a single inst match
    (dolist (module ci-list)
      (let* ((module-name (get-value module 'type))
             (components (get-value module 'components)))

             (dolist (comp components)
               (let* ((component-type-name (first comp) )
                       (component-instance-name (second comp) )
                       (module (third comp ))
                       (ports (fourth comp ))) 
                  (format t "~%--------------------------------~%")
                  (format t "name             : ~a ~%" module-name)
                  (format t "component-type   : ~a ~%" component-type-name)
                  (format t "component-inst  : ~a ~%" component-instance-name)
                  (format t "module (if sub) : ~a ~%" module)
                  (format t "ports-name       : ~a ~%" ports)
                  (dolist (port ports)
                    (format result "~a ~a ~%" (first port)(second port)))))))
    result))


(defun instantiate-coordination-module (type moduleInstanceName)
 (format t "~%=============================================~%")
 (format t "INST CM ~s INSTANCE --> ~s~%" type moduleInstanceName)
 (format t "=============================================~%")

 (let ((module (query-kb *MEMORY* '(is-a type inst-name) `((is-a coordination-module) (type ,type) (inst-name ,moduleInstanceName)))))
   (format t "~%--------------------------------~%")
   (format t "type             : ~a ~%" (get-value module 'type))
   (format t "inst-name        : ~a ~%" (get-value module 'inst-name))
   (format t "comp-interfaces  : ~a ~%" (get-value module 'comp-interfaces))

   (cond
     ((null module)
       (format t "~%=============================================~%")
       (format t "ERROR UNKNOWN MODULE TYPE OR INST")
       (format t "~%=============================================~%"))
     (T
       (dolist (ci (get-value module 'comp-interfaces))
         (let ((ci-inst-name     (concatenate 'string (string (get-value module 'inst-name)) "." (cdr (assoc 'inst-name ci))))
               (ci-type          (cdr (assoc 'type  ci)))
               (ci-services      (cdr (assoc 'services ci)))
               (ci-comp-inst     (cdr (assoc 'component-inst ci))))

               (format t "type             : ~a ~%" ci-type)
               (format t "inst-name        : ~a ~%" ci-inst-name)
               (format t "services         : ~a ~%" ci-services)
               (format t "cm-inst          : ~a ~%" ci-comp-inst)

               (instantiate-coordination-interface ci-type ci-inst-name ci-comp-inst ci-services)))))))
 


(defun format-service-map (services)
  (format t "format-service-map service:~s~%" services)
  (let ((result (make-string-output-stream)))
    ;;there should be only a single inst match
    (dolist (service services)
      (format t "service ~a~%" service)
      (format result "~a ~a ~%" (cdr (assoc 'COORDSERVICE service)) (cdr (assoc 'COMPSERVICE service))))
    (get-output-stream-string result)))

 

(defun instantiate-coordination-interface (type ciInstanceName componentInstanceName services)
 (format t "~%=============================================~%")
 (format t "INST CI ~s INSTANCE --> ~s~%" type ciInstanceName)
 (format t "COMPINST ~s~%" componentInstanceName)
 (format t "SERVICES ~s~%" services)
 (format t "=============================================~%")

 (let ((ptr nil)
       (resultList nil)
       (status  nil)
       (loadmodule-result nil))

   (cffi:with-foreign-strings ((type_str (format nil "~a" type))
                               (ciInstanceName_str (format nil "~a" ciInstanceName))
                               (ciServiceMap_str (format nil "~a" (format-service-map services)))
                               (ciComponentInst_str (format nil "~a" componentInstanceName)))
                                    
     ;;the call to the c function
     (setf ptr (instantiateci type_str ciInstanceName_str ciComponentInst_str ciServiceMap_str)))

   (setf resultList (read-from-string (cffi:foreign-string-to-lisp ptr)))
   (cffi:foreign-free ptr)
  
   (setf status (first resultList))
   (setf loadmodule-result (second resultList))

   (cond
     (( equal status 'ok)
       (format t "=============================================~%")
       (format t "success instantiate coordination interface ~%")
       (format t "=============================================~%~%")
       NIL)
     ((equal status 'error)
       (format t "=============================================~%")
       (format t "~% an error occured: ~s~%" loadmodule-result)
       (format t "=============================================~%~%")
       NIL)
     (T
       (format t "=============================================~%")
       (format t "~% unknown error: ~s~%" loadmodule-result)
       (format t "=============================================~%~%")
       NIL))))



(defun load-coordination-interface (name ci-path)
"This function loads the libs to interface the component coordination interfaces of the 
coordinated functions. The function connects to the c function (loadcoordinationinterface),
which dynamically loads the lib (so/dll) and thereby the symbols."

 (format t "~%=============================================~%")
 (format t "LOAD  COORDINATION  INTERFACE --> ~s~%" name)
 (format t "=============================================~%")

 (let ((ptr nil)
       (resultList nil)
       (status  nil)
       (loadci-result nil))
   (cffi:with-foreign-strings ((name_str (format nil "~a" name))
                               (ci-path_str (format nil "~a" ci-path)))
     ;;the call to the c function
     (setf ptr (loadcoordinationinterface  name_str ci-path_str)))

     (setf resultList (read-from-string (cffi:foreign-string-to-lisp ptr)))
     (cffi:foreign-free ptr)
  
     (setf status (first resultList))
     (setf loadci-result (second resultList))

   (cond
     (( equal status 'ok)
       (format t "=============================================~%")
       (format t "success loadCI ~s~%" name)
       (format t "=============================================~%~%")
       T)

     ((equal status 'error)
       (format t "~% an error occured: ~s~%" loadci-result)
       (format t "=============================================~%~%")
       NIL)
     (T
       (format t "~% unknown error: ~s~%" loadci-result)
       (format t "=============================================~%~%")
       NIL))))


(defun read-coordination-module-system-file (file-name)
"This function reads the system model generated json formated module configuration 
and adds the configuration to the kb"

  (let ((in-string nil)(json-res nil)(module-insts-list nil))
    (setf in-string (with-open-file (in-stream file-name :direction :input)
                        (let ((contents (make-string (file-length in-stream))))
                          (read-sequence contents in-stream)
                          contents)))
    (setf json-res (decode-msg in-string))

    (setf module-insts-list (cdr (assoc 'MODULES-INSTS json-res)))
    (dolist (module-inst-obj module-insts-list)
      (format t "module insts: ~a~%" module-inst-obj)

      (let* ((module-inst     (cdr (assoc 'COORDINATION-MODULE-INST module-inst-obj)))
             (type            (cdr (assoc 'type  module-inst)))
             (ints-name       (intern (string-upcase (cdr (assoc 'inst-name module-inst)))))
             (comp-interfaces-list (cdr (assoc 'COORDINATION-INTERFACES-INSTANCES module-inst))))
        (format t "type:      ~a~%" type)
        (format t "ints-name: ~a~%" ints-name)
        (format t "comp-i:    ~a~%" comp-interfaces-list)
        (update-kb *MEMORY* '(is-a type inst-name) `( (is-a coordination-module) 
          (type ,type) (inst-name ,ints-name) (comp-interfaces ,comp-interfaces-list)))))))


(defun instanciate-all-modules-and-cis (module-path)
  (format t "=======================================~%")
  (format t "AUTO INSTANCE CI-MODULES AND CIs~%")
  (format t "=======================================~%")
  (let* ((modules-list-query (query-kb-all *MEMORY* '(is-a) '((is-a coordination-module))))
         (ci-list nil))

   ;; collect cis and remove duplicates
   (dolist (module modules-list-query)
     (dolist (ci (get-value module 'comp-interfaces))
       (push (cdr (assoc 'type ci)) ci-list)))
   (setq ci-list (remove-duplicates ci-list))
   (setq ci-list (remove (intern (string-upcase *FETCHEVENT-MODULE-NAME*)) ci-list))

   ;; load all cis
   (format t "ci-list: ~a~%" ci-list)
   (dolist (ci ci-list)
     (load-coordination-interface ci module-path))

   ;; inst modules
   (dolist (module modules-list-query)
     (cond
       ((equal-symbol-string (get-value module 'type) *FETCHEVENT-MODULE-NAME*)
         ;;skip
         (format t "Skip fetch event~%"))
       (T
         (format t "Inst Module: ~a - ~a~%" (get-value module 'type) (get-value module 'inst-name))
         (instantiate-coordination-module (get-value module 'type) (get-value module 'inst-name))))))
  (format t "========================================~%")
  (format t "DONE AUTO INSTANCE CI-MODULES AND CIs~%")
  (format t "========================================~%"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; KEEP THE CIs in MEMORY AS WELL - NOT NEEDED SO FAR!

;        (dolist (comp-interfaces comp-interfaces-list)
;;          (format t "comp-interfaces insts: ~a~%" comp-interfaces)
;          (let ((ci-inst-name     (cdr (assoc 'inst-name comp-interfaces)))
;                (ci-type          (cdr (assoc 'type  comp-interfaces)))
;                (ci-services      (cdr (assoc 'services comp-interfaces))))
;;            (format t "ci-inst-name:      ~a~%" ci-inst-name)
;;            (format t "type:              ~a~%" ci-type)
;;            (format t "ci-services:       ~a~%" ci-services)
;            (update-kb *MEMORY* '(is-a type inst-name) `( (is-a coordination-interface) 
;              (type ,ci-type) (inst-name ,ci-inst-name) (services ,ci-services) (coordination-module-inst ,ints-name)))))))))

;;
;; print functions
;;

;(defun show-cis ()
;  (let ((ci-list     (query-kb-all *MEMORY* '(is-a) '((is-a coordination-interface)))))
;    (dolist (ci ci-list)
;      (format t "~%--------------------------------~%")
;      (format t "type             : ~a ~%" (get-value ci 'type))
;      (format t "inst-name        : ~a ~%" (get-value ci 'inst-name))
;      (format t "services         : ~a ~%" (get-value ci 'services))
;      (format t "cm-inst          : ~a ~%" (get-value ci 'coordination-module-inst)))))

;; KEEP THE CIs in MEMORY AS WELL - NOT NEEDED SO FAR!
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun show-modules ()
  (let ((modules-list     (query-kb-all *MEMORY* '(is-a) '((is-a coordination-module)))))
    (dolist (module modules-list)
      (format t "~%--------------------------------~%")
      (format t "type             : ~a ~%" (get-value module 'type))
      (format t "inst-name        : ~a ~%" (get-value module 'inst-name))
      (format t "comp-interfaces  : ~a ~%" (get-value module 'comp-interfaces)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; DEFAULT MODULES 

(defvar *FETCHEVENT-MODULE-NAME* "FETCHEVENTS")
(update-kb *MEMORY* '(is-a type inst-name) `( 
  (is-a coordination-module) 
  (type ,*FETCHEVENT-MODULE-NAME*) 
  (inst-name ,(intern (string-upcase *FETCHEVENT-MODULE-NAME*)))
  (comp-interfaces (((TYPE .  ,(intern (string-upcase *FETCHEVENT-MODULE-NAME*))) (INST-NAME . ,*FETCHEVENT-MODULE-NAME*))))))


(defvar *KB-MODULE-NAME* "KBModule")
(defvar *KB-MODULE-INST-NAME* 'KBMODINST)
;(update-kb *MEMORY* '(is-a type inst-name) `(
;  (is-a coordination-module) 
;  (type ,*KB-MODULE-NAME*) 
;  (inst-name KB) 
;  (comp-interfaces (((TYPE . ,*KB-MODULE-NAME*) (INST-NAME . ,*KB-MODULE-NAME*))))))

;(defvar wiring-map-l '( (wiring   ( (connect "connect")
;                                    (disconnect "disconnect")))))

;(defvar component-map-l '( (shutdownEvent   ( (activate "activate")
;                                    (deactivate "deactivate")))))

;(defvar timer-map-l '( (relative ( (activate "relative-activate")
;                                   (deactivate "relative-deactivate")))
;                       (absolute ( (activate "absolute-activate")
;                                   (deactivate "absolute-deactivate")))))

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;
;; system functions to handle the SmartSoft interface
;;
;;

(defvar *SEQUENCERCLIB* "libComponentTCLSequencer.so")

(cond 
  ((probe-file *SEQUENCERCLIB*) 
   (format t "LOADING ~a from local dir!~%~%" *SEQUENCERCLIB*)
   (load-shared-object *SEQUENCERCLIB*))
  ((probe-file (format nil "~a/bin/~a" (sb-ext:posix-getenv "SMART_ROOT_ACE") *SEQUENCERCLIB*))
   (format t "LOADING ~a from env SMART_ROOT_ACE context!~%~%" *SEQUENCERCLIB*)
   (load-shared-object (format nil "~a/bin/~a" (sb-ext:posix-getenv "SMART_ROOT_ACE") *SEQUENCERCLIB*)))
  ((probe-file (format nil "/opt/smartSoftAce/bin/smartTCL/~a" *SEQUENCERCLIB*)) 
   (format t "LOADING INSTALLED /opt/smartSoftAce/bin/smartTCL/~a~%~%" *SEQUENCERCLIB*)
   (load-shared-object (format nil "/opt/smartSoftAce/bin/smartTCL/~a" *SEQUENCERCLIB*)))

  (T (format t "ERROR loading ~a file not found!~%~%" *SEQUENCERCLIB*)(exit)))


(cffi:defcfun "delay" :void (x :int ))
(cffi:defcfun "command" :pointer (ciType :string) (ciInstance :string) (compType :string) (compInstace :string) (service :string) (param :string))
(cffi:defcfun "initialize" :int (paramFile :string))
(cffi:defcfun "loadcoordinationinterface" :pointer (ciName :string) (ciPath :string))
(cffi:defcfun "instantiateci" :pointer (ciName :string) (ciInstanceName :string) (componentInstanceName :string) (serviceNameMap :string))
(cffi:defcfun "destroyci" :pointer (ciInstanceName :string))

(cffi:defcfun "waitoncompshutdown" :void)
(cffi:defcfun "waitoncomptasktocomplete" :void)
(cffi:defcfun "shutdowncomp" :void)


(setq *SMARTSOFT* (make-instance 'smartsoft))

;;(cond
;  ((probe-file (format nil "~a" (sb-ext:posix-getenv "SMART_ROOT_DEPLOYMENT")))
;   (format t "LOADING interface modules from env SMART_ROOT_DEPLOYMENT context!~%~%")
;   (load-module (format nil "~a/lispInterface" *LISP-INTERFACE-PREFIX*)))
;  ((probe-file (format nil "~a/src/components/SmartLispServerV2/src/lispInterface" (sb-ext:posix-getenv "SMART_ROOT_ACE")))
;   (format t "LOADING interface modules from env SMART_ROOT_ACE context!~%~%")
;   (load-module (format nil "~a/src/components/SmartLispServerV2/src/lispInterface" (sb-ext:posix-getenv "SMART_ROOT_ACE"))))
;  ((probe-file "/opt/smartSoftAce/bin/smartTCL/modules") 
;   (format t "LOADING INSTALLED interface modules /opt/smartSoftAce/bin/smartTCL/modules~%~%")
;   (load-module "/opt/smartSoftAce/bin/smartTCL/modules"))
;  (T (format t "ERROR loading interface modules!~%~%")(exit)))

;; load all skills
;;(load-module "~/SOFTWARE/smartsoft/src/master/smartLispServerV2/lispInterface/skills/skill*.lisp")


(cffi:with-foreign-strings ((args_str (format nil "~{~A~^ ~}" sb-ext:*posix-argv*)))
  (initialize args_str))
