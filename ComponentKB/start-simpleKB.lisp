;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011,2016 Andreas Steck, Matthias Lutz
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


(let ((indexpos nil))
  (setf indexpos  (position "--prefix" sb-ext:*posix-argv* :test #'equal))
  (cond
   ((not (eql indexpos nil))
    (defvar prefix (nth (+ 1 indexpos) sb-ext:*posix-argv*)))))

(format t "Prefix: ~a~%" prefix)

;; cffi via quicklisp
;(let ((quicklisp-init (merge-pathnames "quicklisp/setup.lisp"
;                                       (user-homedir-pathname))))
;  (when (probe-file quicklisp-init)
;    (load quicklisp-init)))
;(ql:quickload "cffi")

(require "asdf")
(require "cffi")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; cl-json

 ;; cl-json
  (setf asdf:*central-registry*
       (list* '*default-pathname-defaults*
              (format NIL "~a/cl-json/" prefix)
              (format NIL "./cl-json/")
              asdf:*central-registry*))

(require "cl-json")

(defun keep-lisp-symbol-string (string)
  "Take a symbol name as a string and keep it lowercase"
  (string-downcase string))

(defun encode-msg (input-alist)
  "This function takes an alist and encodes it to a json object"
  (let ((json-msg nil))
    (format t "encode alist: ~a~%" input-alist)

    (handler-case
        (let ((cl-json:*lisp-identifier-name-to-json* #'keep-lisp-symbol-string)
              (str (make-string-output-stream)))
              (cl-json:encode-json input-alist str)
              (setf json-msg (get-output-stream-string str)))
      (t (c)
        (format t "[encode-msg] WARNING decode msg json vaild erro: ~a~%" c)
        (setf json-msg nil)
        (values nil c)))
    
    (format t "encode-msg encoded-msg: ~a~%" json-msg)
;; this is just doing (with-output-to-string!!
;;    (format t "encode-json-to-string msg ~a~%" (cl-json:encode-json-to-string json-msg))
    json-msg))

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; load SmartTCL
(load (format nil "~a/smartSimpleKB_lisp/load-simpleKB.lisp" prefix))
(load-smartsimplekb prefix)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; M E M O R Y     - - -     K N O W L E D G E B A S E

;; kb-update
(defun kb-update (&key key value)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (update-kb *MEMORY* key value))
    (memorychanged)
  nil)

;; kb-update-batch perform multiple updates before checking for changes
;; optional params are used to delete entries before applying the changes (no event check is applied in between)
(defun kb-update-batch (&key updates-list (delete-key nil delete-key-supplied-p) (delete-value nil))
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (if delete-key-supplied-p (delete-kb *MEMORY* delete-key delete-value))
    (dolist (update updates-list)
      (update-kb *MEMORY* (first update) (second update))))
    (memorychanged)
  nil)

;; kb-delete
(defun kb-delete (&key key value)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (delete-kb *MEMORY* key value))
    (memorychanged)
  nil)


;;TODO why is there no lock?
;; kb-query
(defun kb-query (&key key value)
  (query-kb *MEMORY* key value))



;; kb-query-all
(defun kb-query-all (&key key value)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (query-kb-all *MEMORY* key value)))

;; kb-add-entry
(defun kb-add-entry (values)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (add-kb-entry *MEMORY* values))
    (memorychanged))


;; kb-register-chained-entry
(defun kb-register-chained-entry (&key key value)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (register-chained-entry *MEMORY* key value)))

;; kb-update-from-master-entry
(defun kb-update-from-master-entry (&key id entries)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (update-from-master-entry *MEMORY* id entries))
    (memorychanged))

;; serialize-kb-query-answer
(defun serialize-answer (result)
  (let ((answer nil))
    (cond
      ((atom result)
        (cond 
          ((typep result 'KB-ENTRY)
            ;(format t "The result is a SINGLE KB Entry!~%")
            (setf answer (get-kb-entry-serialize result)))
          (T
            ;(format t "The result is not a KB Entry!~%")
            (setf answer result))))
      (T
        ;(format t "The result is NOT a atom!~%")
        (dolist (res result)
              (setf answer (append answer (list (get-kb-entry-serialize res)))))))
    answer))

;; function uses hashes to check if entries are equal
(defun compare-state-with-kb-entries (state entries)
  (let ((state-hashs nil)
        (entries-hashs nil)
        (entries-s (serialize-answer entries)))
;     (format t "compare-state-with-kb-entries ~%")
     ;;make sure that entries-s is a list in any case
     (cond
      ((atom entries-s)
        (cond 
          ((typep entries-s 'KB-ENTRY)
;            (format t "MATTHIAS The result is a SINGLE KB Entry!~%")
            (setf entries-s `(,entries-s)))
          (T
;            (format t "THIS SHOULD NOT HAVE HAPPEND?~%")
            (setf entries-s `(,entries-s))))))
    ;;first check if size of state equals size of entries
    (if (equal (length state) (length entries-s))
     () ;;equal size --> further checks needed!
     ;(progn (format t "State and current res have different size --> not equal~%")
            (return-from compare-state-with-kb-entries nil))
    ;(setf state-hashs (loop for s in state collect (sxhash (write-to-string s))))
    ;(setf entries-hashs (loop for s in entries-s collect (sxhash (write-to-string s))))

    ;loop over each entry; loop over each value
    (setf state-hashs (sort (loop for s in state collect (sxhash (write-to-string (sort (loop for ss in s collect (sxhash (write-to-string ss))) #'<)))) #'<))
    (setf entries-hashs (sort (loop for s in entries-s collect (sxhash (write-to-string (sort (loop for ss in s collect (sxhash (write-to-string ss))) #'<)))) #'<))

;    (format t "state-hashes  :~a~%" state-hashs)
;    (format t "entries-hashs :~a~%" entries-hashs)
;    (format t "set diff      :~a~%" (set-difference state-hashs entries-hashs ))

    (null (set-difference state-hashs entries-hashs))))


;; kb-event
(defun kb-event (&key query formating state)
  (SB-THREAD:with-recursive-lock (*MEMORY-LOCK*)
    (let ((query-result nil) (answer '(nil nil)))
;      (format t "kb-event query: ~s  state: ~s~%" query state)

      ;;query for new state!
      (setf query-result (eval query))
      ;(format t "NEW Query Result: ~s~%" query-result)
      (cond
        ((null state)
          (if (null query-result)
            (setf answer '('(nil nil nil)))
            (setf answer `(,(serialize-answer query-result) T
                           ,(if (null formating) 
                             nil 
                             (handler-case
                               (funcall (eval formating) query-result)
                               (t (c)
                                 (format t "[EVENT-L] ERROR formating the result: ~a~%" c)
                                 (setf result nil)
                                 (values nil c))))))))
        (T
          (if (compare-state-with-kb-entries state query-result)
            (setf answer '(nil nil nil))
            (setf answer `(,(serialize-answer query-result) T 
                           ,(if (null formating) 
                             nil 
                            (handler-case
                               (funcall (eval formating) query-result)
                               (t (c)
                                 (format t "[EVENT-L] ERROR formating the result: ~a~%" c)
                                 (setf result nil)
                                 (values nil c)))))))))
      answer)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; KB-Event thread
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun check-events-loop ()
  (loop 
    (let ((check-event nil) (event-query nil) (event-formating) (event-state nil) (answer nil))
      (format t "[EVENT-L] Check events loop~%")
      (setf check-event (read-from-string (get-check-param-event)))
      (setf event-query (first check-event))
      (cond
        ((equal event-query 'ON_COMPONENT_SHUTDOWN)
         (format t "[EVENT-L] ON_COMPONENT_SHUTDOWN in lisp~%")
         (return-from check-events-loop)))
      (setf event-formating (second check-event))
      (setf event-state (third check-event))
      (format t "[EVENT-L] Check changes event query: ~s ~%" event-query)
;      (format t "Got event query: ~s format: ~s state: ~s~%" event-query event-formating event-state)
      (setf answer (kb-event :query event-query :formating event-formating :state event-state))
;      (format t "[EVENT-L] Result -  FIRE: ~a answ: ~a~%~%" (second answer) (first answer))
      (format t "[EVENT-L] Result -  FIRE: ~a ~%~%" (second answer))
      (answer-check-event-param (first answer) (second answer) (third answer)))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(format t "Start check-event-thread~%")
(setf *check-events-thread* (SB-THREAD:make-thread #'check-events-loop))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ON_COMPONENT_SHUTDOWN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun ON_COMPONENT_SHUTDOWN ()
 "THIS IS CALLED VIA THE MESSAGE IF THE COMPONENT SHUTSDOWN!"
 (format t "ON_COMPONENT_SHUTDOWN --> LISP ~%")
 'SHUTDOWN)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; main thread
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(block main-loop
 (loop 
  (let ((request nil)
    (result nil)
    (answer nil)
    (querycommand nil))
    (setf request (get-query))
    (setf querycommand (read-from-string request))
    (format t "[MAIN-L] id:~a - Got query request: ~a~%" (first querycommand) request)

    (handler-case
      (setf result (eval (second querycommand)))
      (t (c)
        (format t "[MAIN-L] ERROR evaluating the KB Query: ~a~%" c)
        (setf result nil)
        (values nil c)))

    ;(setf result (eval (second querycommand)))

    ;;(setf result (funcall (symbol-function (first querycommand)) :key (second querycommand) :value (third querycommand)))

    (cond
      ((atom result)
        (cond 
          ((typep result 'KB-ENTRY)
;            (format t "[MAIN-L] The result is a SINGLE KB Entry!~%")
            (setf answer (get-kb-entry-serialize result)))
          ((equal result 'SHUTDOWN)
            (format t "[MAIN-L] SHUTDOWN --> BREAK~%")
            (return-from main-loop))
          (T
;            (format t "[MAIN-L] The result is not a KB Entry!~%")
            (setf answer result))))
      (T
;        (format t "[MAIN-L] The result is NOT a atom!~%")
        (dolist (res result)
              (setf answer (append answer (list (get-kb-entry-serialize res)))))))
;    (format t "[MAIN-L] Query result id:~a answ:~s ~%~%" (first querycommand) answer)
    (format t "[MAIN-L] id:~a Query result.~%~%" (first querycommand))
    (answer-query answer (first querycommand)))))

(format t "[main] join-thread:  check-events-loop ... ~%")
(SB-THREAD:join-thread *check-events-thread*)
(format t "[main] ...DONE ~%")
(format t "[main] join-thread:  component... ~%")
(waitoncomptasktocomplete)
(format t "[main] ...DONE ~%")

(format t "QUIT.~%")
(quit)

