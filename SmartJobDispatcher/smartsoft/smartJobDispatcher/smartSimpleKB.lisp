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


;;; internal-remove <instance> <value>
;;;   description : remove the entry belonging to <value>
;;;   example     : (internal-remove *a* '(id))
;;;
;;; internal-add <instance> <value>
;;;   description : add the <value> without further checks to the entry
;;;   example     : (internal-add *a* '(id 5))
;;;
;;; check-key <instance> <key> <value>
;;;   description : checks whether the key values match the instance and returns
;;;                 T or NIL
;;;   example     : (check-key *a* '(id) '((id 5)(color red)))
;;;
;;; get-value <instance> <slot>
;;;   description : returns the value from the specified slot of this instance
;;;   example     : (get-value *a* '(id))

;;(make-package :SimpleKB)
;;(use-package 'SimpleKB)

(defgeneric internal-remove (instance value))
(defgeneric internal-add (instance value))
(defgeneric show (instance))
(defgeneric check-key (instance key values))
(defgeneric get-value (instance slot))
(defgeneric internal-reset (instance))
(defgeneric internal-check (instance key values))
(defgeneric delete-kb (instance key values))
(defgeneric update-kb (instance key values))
(defgeneric update-kb-all (instance key values))
(defgeneric query-kb (instance key values)) 
(defgeneric query-kb-all (instance key values))

(defgeneric get-kb-entry-serialize (instance))
(defgeneric add-kb-entry (instance values))

;method for calls with nil
(defmethod get-kb-entry-serialize ((instance NULL))
  nil)

(defclass KB-ENTRY ()
  ((data :accessor kb-entry-data :initarg :data :initform nil)))

(defmethod internal-remove ((instance kb-entry) value)
  (let* ((atmp (assoc (first value) (kb-entry-data instance)))
         (result (loop for tmp in (kb-entry-data instance) when (not (equal tmp atmp)) collect tmp)))
    (setf (kb-entry-data instance) result)))

(defmethod internal-add ((instance kb-entry) value)
  (push value (kb-entry-data instance)))

(defmethod show ((instance kb-entry))
  (format t "~s~%" (kb-entry-data instance)))

(defmethod get-kb-entry-serialize ((instance kb-entry))
  (kb-entry-data instance))

(defmethod check-key ((instance kb-entry) key values)
  (let ((result T))
    (dolist (tmpkey key result)
      (cond ((not (equal (get-value instance tmpkey) (second (assoc tmpkey values))))
             (setq result nil))))))
    
(defmethod get-value ((instance kb-entry) slot)
  (second (assoc slot (kb-entry-data instance))))

; this generic method is necessary to avoid problems when accessing slot values of an unsuccessful query-kb
(defmethod get-value (instance slot)
  nil)

;;; syntax
;;;     key      slot names whose values must match. NIL means any object
;;;              since nothing must match !
;;;     values   values of slots
;;;              A slot value is always an identifier/value pair
;;;     example  '(id color) '((id 5)(color red)(pos (10000 12000 90)))
;;;              The values of the slots <id> and <color> must match
;;;              <5> and <red> to update the pos-slot of that instance 
;;;
;;; internal-reset <instance>
;;;   description : simply deletes the kb-memory slot and therefore deletes the 
;;;                 knowledge base
;;;   example     : (internal-reset *a*)
;;;
;;; internal-check <instance> <key> <values>
;;;   description : check whether the specified entry already exists and return the
;;;                 first matching instance or NIL
;;;   example     :
;;;
;;; delete-kb <instance> <key> <values>
;;;   description : delete all matching instances
;;;   example     : (delete-kb *a* '(id) '((id 2))
;;;
;;; update-kb <instance> <key> <values>
;;;   description : update the knowledge base by overwriting the first matching entry.
;;;                 If no such entry exists, create a new instance.
;;;                 If nil is given as slot value, remove this slot
;;;   example     : (update-kb *a* '(id color) '((id 2)(color red)(class ball)))    first matching instance: add/overwrite slot 'class, slots 'id and 'color must match
;;;                 (update-kb *a* '(id color) '((id 2)(color red)(class nil)))     first matching instance: remove slot 'class, slots 'id and 'color must match
;;;                 (update-kb *a* '() '(color red))                                add/overwrite slot 'color of first instance found
;;;                 (update-kb *a* '() '(color nil))                                remove slot 'color of first instance found
;;;
;;; update-kb-all <instance> <key> <values>
;;;   description: same as update-kb but is applied to all relevant instances
;;;                update the knowledge base by overwriting all matching entries
;;;                if no matching entry exists, create a new instance
;;;                if nil is given as slot value, remove this slot from all matching instances
;;;   example    : (update-kb *a* '() '((measured nil)))     remove the slot 'measured from all instances
;;;
;;; query-kb <instance> <key> <values>
;;;   description : returns first matching instance
;;;   example     : (query-kb *a* '(id color) '((id 2)(color red)))
;;;
;;; query-kb-all <instance> <key> <values>
;;;   description : returns a list of matching instances
;;;   example     : (query-kb-all *a* '(color) '((color red)))

(defclass KB-MAIN-CLASS ()
  ((memory :accessor kb-memory :initform nil)))

(defmethod internal-reset ((instance kb-main-class))
  (setf (kb-memory instance) nil))

(defmethod internal-check ((instance kb-main-class) key values)
  (let ((result nil))
    (dolist (entry (kb-memory instance) result)
      (cond ((and (null result) (check-key entry key values))
             (setq result entry))))))

(defmethod show ((instance kb-main-class))
  (dolist (tmp (kb-memory instance))
    (show tmp)))

(defmethod add-kb-entry ((instance kb-main-class) values)
  (push (make-instance 'kb-entry :data values) (kb-memory instance)))

(defmethod delete-kb ((instance kb-main-class) key values)
  (loop
    (let ((result (internal-check instance key values)))
      (setf (kb-memory instance) (delete result (kb-memory instance)))
      (when (null result) (return)))))

; (defmethod update-kb ((instance kb-main-class) key values)
;   (let ((entry (internal-check instance key values)))
;     (cond ((null entry)
;              (push (make-instance 'kb-entry :data values) (kb-memory instance)))
;           (T (dolist (value values)
;                (format t "value ~s~%" value)
;                (cond ((null (get-value entry (first value)))
;                         (internal-add entry value))
;                      (T (format t "kb data ~s~%" (kb-entry-data entry))
;                         (internal-remove entry (list (first value)))
;                         (internal-add entry value))))))))  

(defmethod update-kb ((instance kb-main-class) key values)
  (let ((entry (internal-check instance key values)))
    (cond ((null entry)
             (push (make-instance 'kb-entry :data values) (kb-memory instance)))
          (T (dolist (value values)
               (cond ((and (null (get-value entry (first value))) (not (null (second value))))
                        ; new slot and slot value given, therefore add it
                        (internal-add entry value))
                     ((and (not (null (get-value entry (first value)))) (not (null (second value))))
                        ; slot already exists and new value is given, therefore delete old entry and add new value
                        (internal-remove entry (list (first value)))
                        (internal-add entry value))
                     ((null (second value))
                        ; new value is nil, therefore remove this slot if it already existed
                        (internal-remove entry (list (first value))))
                     (T (format t "kb data ~s~%" (kb-entry-data entry))
                        ; meaningless combination, do nothing
                        )))))))  

(defmethod update-kb-all ((instance kb-main-class) key values)
  (let ((matching (query-kb-all instance key values)))
    (cond ((null matching)
             ; no matching instance at all
             (push (make-instance 'kb-entry :data values) (kb-memory instance)))
          (T 
             ; found matching instances, therefore apply update to those instances
             (dolist (entry matching)
               (dolist (value values)
                 (cond ((and (null (get-value entry (first value))) (not (null (second value))))
                          ; new slot and slot value given, therefore add it
                          (internal-add entry value))
                       ((and (not (null (get-value entry (first value)))) (not (null (second value))))
                          ; slot already exists and new value is given, therefore delete old entry and add new value
                          (internal-remove entry (list (first value)))
                          (internal-add entry value))
                       ((null (second value))
                          ; new value is nil, therefore remove this slot if it already existed
                          (internal-remove entry (list (first value))))
                       (T (format t "kb data ~s~%" (kb-entry-data entry))
                          ; meaningless combination, do nothing
                          ))))))))
 
(defmethod query-kb ((instance kb-main-class) key values)
  (internal-check instance key values))

(defmethod query-kb-all ((instance kb-main-class) key values)
  (let ((result nil))
    (dolist (entry (kb-memory instance) result)
      (cond ((check-key entry key values) 
        (push entry result))))))

