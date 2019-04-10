
;;lisp/c++ interface
(defvar *check-events-thread* nil)
(defvar *CHAINED-ENTRIES-ALIST* nil)

;;KB
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

;for slaved entries
(defgeneric register-chained-entry (instance key values))
(defgeneric update-from-master-entry (instance id entries))
