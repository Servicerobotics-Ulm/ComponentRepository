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

(declaim (ftype (function (&optional string string) t) compile-and-load-smartjobdispatcher))

(require "asdf")
(require "cffi")

(let ((indexpos nil)
  (jobdist-prefix nil)
  (lispinterface-prefix))

  (setf indexpos  (position "--jobdist-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf jobdist-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))
  (setf indexpos  (position "--lispinterface-prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (setf lispinterface-prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*))))

  (format t "all: ~s~%" sb-ext:*posix-argv*)
  (format t "Prefix: ~a~%" jobdist-prefix)
  (format t "lisp-Prefix: ~a~%" lispinterface-prefix)

  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/parser.ini/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/let-plus/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/more-conditions-20170227-git/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/esrap/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/architecture.builder-protocol/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/parser.common-rules/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/anaphora/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/closer-mop/" jobdist-prefix) asdf:*central-registry*))
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/split-sequence/" jobdist-prefix) asdf:*central-registry*))
  
  (setf asdf:*central-registry* (list* '*default-pathname-defaults* (format NIL "~a../lisp_libs/cl-json/" jobdist-prefix) asdf:*central-registry*))
  
  (require "cl-json")

  (require "parser.ini")


  ;; load SmartTCL
  (load (format nil "~aload-smartJobDispatcher.lisp" jobdist-prefix))
  (load (format nil "~a../config-jobdispatcher.lisp" jobdist-prefix))
  (compile-and-load-smartjobdispatcher jobdist-prefix lispinterface-prefix))


;; J O B -  C L A S S
(tcl-kb-update 
  :key '(is-a type) 
  :value '( 
            (is-a job-class)
            (id nil)
            (state nil)
            (error-state nil)
            (priority nil)
            (robotid nil)
            (manual-assigned nil)
            (start-location nil)
            (start-belt nil)
            (start-manual nil)
            (end-location nil)
            (end-belt nil)
            (end-manual nil)
            (type DeliverFromTo)))


;; F L E E T  -  C L A S S
(tcl-kb-update 
  :key '(is-a id) 
  :value '( 
            (is-a fleet-class)
            (id 0)
            (state nil)
            (robots nil)
            (robot-mapping nil)))


;; F L E E T   S T A T E S
;; INIT
;; SHUTDOWN
;; SETUP
;; MAPPING
;; OPERATIONAL
;; FATALERROR





;; J O B
;(tcl-kb-update  :key '(is-a id) :value `( (is-a job) (id 1) (state NOTSTARTED) (error-state NOERROR) (priority 0) (robotid nil) (start-location 1) (start-belt 1)  (end-location 2) (end-belt 3) (type DELIVER-FROM-TO)))

;; FLEET --> INIT
(tcl-kb-update :key '(is-a id) :value '((is-a fleet-class)(id 0)(state INIT)))


(handler-case
  (progn
;; check if all master components are up an running!
    (tcl-life-cycle-wait-for-state :server 'symbolicplanner :state "Neutral")
    (tcl-life-cycle-wait-for-state :server 'highlevelcommand :state "Neutral")
    (tcl-life-cycle-wait-for-state :server 'knowledgebase :state "Neutral"))
  ;  (tcl-life-cycle-wait-for-state :server 'pathnavigationserver :state "Neutral")
  ;  (tcl-life-cycle-wait-for-state :server 'fileprovider :state "Neutral")
  ;  (tcl-life-cycle-wait-for-state :server 'mapper :state "Neutral")
    ;(tcl-life-cycle-wait-for-state :server 'robotstateviewer :state "Neutral")
   ; (tcl-life-cycle-wait-for-state :server 'robotinorpcbridge :state "Neutral"))
;;    (tcl-life-cycle-wait-for-state :server 'navigationcoordinationserver :state "Neutral")
  (component-life-cycle-error (se) 
  (progn (format t "Error in component life-cycle-setting ~a --> QUIT~%" (text se))
         (exit))))


(format t "LOAD STATIONS TYPES FROM FILE~%")

(cond
  ((not (equal (probe-file "/etc/robotino/station-types.conf") nil))
    (format t "data_master/maps/default dir is there!~%")
    (let ((parser.ini:*assignment-operator*                     #\=)
          (parser.ini:*value-terminating-whitespace-expression* #\Newline)
          (read-res nil))
      
      (handler-case
        (setf read-res (parser.ini:parse #P"/etc/robotino/station-types.conf" 'list))
      (parser.ini:ini-parse-error (se)
      (progn (format t "Error parsing station type config file ~a --> QUIT~%" (text se))
         (exit))))
 

  (dolist (item read-res)
    (let ((entry-alist nil)
          (entry-name nil))
      ;(format t "item:~s~%" item)
      (setf entry-name (read-from-string (first (getf item :name))))
      (format t "PList name: ~s~%" entry-name)

      (destructuring-bind (&key section &allow-other-keys) item
        (dolist (option (getf section :section-option))
          ;(format t "Option: ~s~%" option)
          (destructuring-bind (&key name value &allow-other-keys) (first option)
            (progn  ;(format t "Option name: ~s value:~s ~%" name value)
                    (push (list (read-from-string (first name)) value) entry-alist)))))

      (format t "entry-alist: ~s~%" entry-alist)

      (multiple-value-bind (min-station-width max-station-width offset-between-center-of-belts offset-left-reflector-to-1stBelt-center ir-dock-stop-dist-l2-min
                            ir-dock-stop-dist-l2-max ir-dock-stop-dist-l1 ir-dock-center-sensor-zero docking-type commincation-type number-of-belts 
                            max-reflector-dist laser-dock-stop-dist description) 
                           (values (read-from-string (second (assoc 'min-station-width entry-alist )))
                                   (read-from-string (second (assoc 'max-station-width entry-alist )))
                                   (read-from-string (second (assoc 'offset-between-center-of-belts entry-alist)))
                                   (read-from-string (second (assoc 'offset-left-reflector-to-1stBelt-center entry-alist )))
                                   (read-from-string (second (assoc 'ir-dock-stop-dist-l2-min entry-alist )))
                                   (read-from-string (second (assoc 'ir-dock-stop-dist-l2-max entry-alist )))
                                   (read-from-string (second (assoc 'ir-dock-stop-dist-l1 entry-alist )))
                                   (read-from-string (second (assoc 'ir-dock-center-sensor-zero entry-alist )))
                                   (read-from-string (second (assoc 'docking-type entry-alist )))
                                   (read-from-string (second (assoc 'commincation-type entry-alist )))
                                   (read-from-string (second (assoc 'number-of-belts entry-alist )))
                                   (read-from-string (second (assoc 'max-reflector-dist entry-alist )))
                                   (read-from-string (second (assoc 'laser-dock-stop-dist entry-alist )))
                                   (second (assoc 'description entry-alist )))


      (format t "min-station-width: ~s~%" min-station-width)
      (format t "max-station-width: ~s~%" max-station-width)
      (format t "offset-between-center-of-belts: ~s~%" offset-between-center-of-belts)
      (format t "offset-left-reflector-to-1stBelt-center: ~s~%" offset-left-reflector-to-1stBelt-center)
      (format t "ir-dock-stop-dist-l2-min: ~s~%" ir-dock-stop-dist-l2-min)
      (format t "ir-dock-stop-dist-l2-max: ~s~%" ir-dock-stop-dist-l2-max)
      (format t "ir-dock-stop-dist-l1: ~s~%" ir-dock-stop-dist-l1)
      (format t "ir-dock-center-sensor-zero: ~s~%" ir-dock-center-sensor-zero)
      (format t "docking-type: ~s~%" docking-type)
      (format t "commincation-type: ~s~%" commincation-type)
      (format t "number-of-belts: ~s~%" number-of-belts)
      (format t "max-reflector-dist: ~s~%" max-reflector-dist)
      (format t "laser-dock-stop-dist: ~s~%" laser-dock-stop-dist)
      (format t "description: ~s~%" description)

      (tcl-kb-update 
       :key '(is-a name) 
       :value `(
           (is-a station-type)
           (name ,entry-name)
           (min-station-width ,min-station-width)
           (max-station-width ,max-station-width)
           (offset-between-center-of-belts ,offset-between-center-of-belts)
           (offset-left-reflector-to-1stBelt-center ,offset-left-reflector-to-1stBelt-center)
           (ir-dock-stop-dist-l2-min ,ir-dock-stop-dist-l2-min)
           (ir-dock-stop-dist-l2-max ,ir-dock-stop-dist-l2-max)
           (ir-dock-stop-dist-l1 ,ir-dock-stop-dist-l1)
           (ir-dock-center-sensor-zero ,ir-dock-center-sensor-zero)
           (commincation-type ,commincation-type)
           (number-of-belts ,number-of-belts)
           (docking-type ,docking-type)
           (max-reflector-dist ,max-reflector-dist)
           (laser-dock-stop-dist ,laser-dock-stop-dist)
           (description ,description))))))))

  (T
    (format t "ERROR NOT ABLE TO FIND STATION TYPE CONFIG ~%")))


  (format t "CHECK IF data_master/maps/default is there~%")
  (cond
    ((not (equal (probe-file "/opt/smartsoft/data_master/maps/default") nil))
                     (format t "data_master/maps/default dir is there!~%"))
    (T
      (format t "INFO Not able to find data_master/maps/default dir --> INI STATE! ~%")))
      ;(sb-ext:run-program "/bin/mkdir" '("/opt/smartsoft/data/maps/master-sync") :wait T)))
      ;(sb-ext:run-program "/bin/ln" '("-s" "/opt/smartsoft/data/maps/slave-map-dir" "/opt/smartsoft/data/maps/default") :wait T)


  (format t "LOAD POSTIONS FROM FILE~%")
  (cond
   ((eql (sb-ext:posix-getenv "SMART_ROOT_ACE") nil)
     (format t "Load position list system context~%")
     (with-open-file (str "/opt/smartSoftAce/data_master/maps/default/positions.lisp" :if-does-not-exist nil)
     (if str
       (loop for entry = (read str nil)
       while entry do (tcl-kb-add-entry entry)))))
                 
   (T
     (format t "Load position list from SMART_ROOT_ACE context~%")
     (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/positions.lisp") :if-does-not-exist nil)
     (if str
       (loop for entry = (read str nil)
         while entry do (tcl-kb-add-entry entry))))))

  (format t "LOAD STATIONS FROM FILE~%")
  (cond
   ((eql (sb-ext:posix-getenv "SMART_ROOT_ACE") nil)
     (format t "Load stations list system context~%")
     (with-open-file (str "/opt/smartSoftAce/data_master/maps/default/stations.lisp" :if-does-not-exist nil)
     (if str
       (loop for entry = (read str nil)
       while entry do (tcl-kb-add-entry entry)))))
                 
   (T
     (format t "Load stations list from SMART_ROOT_ACE context~%")
     (with-open-file (str (concatenate 'string (sb-ext:posix-getenv "SMART_ROOT_ACE") "/data_master/maps/default/stations.lisp") :if-does-not-exist nil)
     (if str
       (loop for entry = (read str nil)
         while entry do (tcl-kb-add-entry entry))))))

  (format t "LOAD DEFAULT MAP FROM FILE~%")
  (tcl-param :server 'mapper :slot 'COMMNAVIGATIONOBJECTS.MAPPERPARAMS.LTMLOADYAML :value "navigation-map")
  
  ;(tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.SET_LOCALIZATION_TYP :value `(LOCALIZATION -1))
  (tcl-state :server 'robotinorpcbridge :state "Deactivated")
  (tcl-param :server 'robotinorpcbridge  :slot 'COMMROBOTINOOBJECTS.ROBOTINORPCPARAMETER.REFRESH_STATIC_MAP)
  (tcl-state :server 'robotinorpcbridge :state "staticMap")

(format t "INIT STATE:~%")
(show-robots)
(show-jobs)
(format t "RUNNING~%")


;; FLEET --> OPERATIONAL
(tcl-kb-update :key '(is-a id) :value '((is-a fleet-class)(id 0)(state OPERATIONAL)))

;(defparameter *manual-jobs-list* nil
;  "List of manual josb.")
;(defparameter *auto-jobs-list* nil
;  "List of auto jobs.")
;push job id to list to get job order
;(if (equal (fifth request) -1) nil 
;    (push (third request) (cdr (last *auto-jobs-list*))))

(defvar nmbr nil)

(defun menu()
(loop
  (format t "MENU~%====~%~%")
  (format t " 1 - runJobDispachter ~%")
  (format t "99 - quit menu ~%")
  (format t " 0 - exit ~%")
  (setf nmbr (parse-integer (read-line nil)))
  (format t "your choice: ~d ~%" nmbr)
  
  (cond
    ((equal nmbr 1)
      (runJobDispachter))
    ((equal nmbr 99)
      (format t "type (menu) to come back to menu ~%")
      (return))
    ((equal nmbr 0)
      (format t "QUIT.~%")
      (exit)))))


(cond
 ((equal (runJobDispachter) 'quit)
  (format t "[main] join-thread:  component... ~%")
  (waitoncomptasktocomplete)
  (format t "[main] ...DONE ~%")
  (format t "QUIT.~%")
  (exit))
 (T
  (format t "No quit exit of loop~%")))


