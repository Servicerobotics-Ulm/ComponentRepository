(defun decode-msg (input-msg)
  "This function takes a string in jsonformat and decodes it to a alist"
  (let ((parsed-msg nil))
    (format t "Decode msg: ~a~%" input-msg)
    (format t "Typeof msg: ~a~%" (type-of input-msg))

    (handler-case
      (with-input-from-string (s input-msg)
        (let ((cl-json:*json-symbols-package* nil))
          (setf parsed-msg (cl-json:decode-json s))))
      (t (c)
        (format t "[decode-json-msg] ERROR decode msg json invalid : ~a~%" c)
        (setf parsed-msg nil)
        (values nil c)))

    
    (format t "Parsed result ~s~%" parsed-msg)
    parsed-msg))

(defun decode-msg-from-file (filename)
  "This function takes a file-path in jsonformat and decodes it to a alist"
  (let ((parsed-msg nil))
    (format t "Decode file: ~a~%" filename)
    (format t "Typeof msg: ~a~%" (type-of filename))

    (handler-case
      (with-open-file (s filename)
        (let ((cl-json:*json-symbols-package* nil))
          (setf parsed-msg (cl-json:decode-json s))))
      (t (c)
        (format t "[decode-json-msg] ERROR decode file: ~a~%" c)
        (setf parsed-msg nil)
        (values nil c)))

    
    (format t "Parsed result ~s~%" parsed-msg)
    parsed-msg))


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
    
    (format t "encode-msg msg: ~a~%" json-msg)
;; this is just doing (with-output-to-string!!
;;    (format t "encode-json-to-string msg ~a~%" (cl-json:encode-json-to-string json-msg))
    json-msg))
