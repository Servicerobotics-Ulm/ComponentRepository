;;;; Copyright (c) 2007-2013 Nikodemus Siivola <nikodemus@random-state.net>
;;;; Copyright (c) 2012-2019 Jan Moringen <jmoringe@techfak.uni-bielefeld.de>
;;;;
;;;; Permission is hereby granted, free of charge, to any person
;;;; obtaining a copy of this software and associated documentation files
;;;; (the "Software"), to deal in the Software without restriction,
;;;; including without limitation the rights to use, copy, modify, merge,
;;;; publish, distribute, sublicense, and/or sell copies of the Software,
;;;; and to permit persons to whom the Software is furnished to do so,
;;;; subject to the following conditions:
;;;;
;;;; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
;;;; EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
;;;; MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
;;;; IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
;;;; CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
;;;; TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
;;;; SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

(cl:in-package #:esrap)

;;; Miscellany

(defun text (&rest arguments)
  "Arguments must be strings, or lists whose leaves are strings.
Catenates all the strings in arguments into a single string."
  (with-output-to-string (s)
    (labels ((cat-list (list)
               (dolist (elt list)
                 (etypecase elt
                   (string (write-string elt s))
                   (character (write-char elt s))
                   (list (cat-list elt))))))
      (cat-list arguments))))

(defun singleton-option (context form keyword type &key default)
  (let ((value default)
        (value-seen nil))
    (lambda (&optional (new-value nil new-value-p))
      (cond
        ((not new-value-p)
         value)
        ((not (typep new-value type))
         (error 'simple-type-error
                :datum new-value
                :expected-type type
                :format-control "~@<The value ~S is not a valid ~
                                 argument to the ~S ~S option.~@:>"
                :format-arguments (list new-value keyword context)))
        (value-seen
         (error "~@<Multiple ~S options in ~S form:~@:_~@:_~
                 ~2@T~S.~@:>"
                keyword context form))
        (t
         (setf value-seen t
               value new-value))))))

;;; http://jcsu.jesus.cam.ac.uk/~csr21/papers/features.pdf
(eval-when (:compile-toplevel :execute)
  (when (and (find-package '#:sb-ext)
             (find-symbol (string '#:with-current-source-form) '#:sb-ext))
    (pushnew 'sb-ext-with-current-source-form *features*)))

(defmacro with-current-source-form ((&rest forms) &body body)
  #-esrap::sb-ext-with-current-source-form (declare (ignore forms))
  #+esrap::sb-ext-with-current-source-form
  `(sb-ext:with-current-source-form (,@forms) ,@body)
  #-esrap::sb-ext-with-current-source-form
  `(progn ,@body))

;;; DEFRULE support functions

(defun parse-lambda-list-maybe-containing-&bounds (lambda-list)
  "Parse &BOUNDS section in LAMBDA-LIST and return three values:

1. The standard lambda list sublist of LAMBDA-LIST
2. A symbol that should be bound to the start of a matching substring
3. A symbol that should be bound to the end of a matching substring
4. A list containing symbols that were GENSYM'ed.

The second and/or third values are GENSYMS if LAMBDA-LIST contains a
partial or no &BOUNDS section, in which case fourth value contains them
for use with IGNORE."
  (let ((length (length lambda-list))
        (index  (position '&bounds lambda-list)))
    (multiple-value-bind (lambda-list start end gensyms)
        (cond
          ;; Look for &BOUNDS START END.
          ((eql index (- length 3))
           (values (subseq lambda-list 0 index)
                   (nth (+ index 1) lambda-list)
                   (nth (+ index 2) lambda-list)
                   '()))
          ;; Look for &BOUNDS START.
          ((eql index (- length 2))
           (let ((end (gensym "END")))
             (values (subseq lambda-list 0 index)
                     (nth (+ index 1) lambda-list)
                     end
                     (list end))))
          ;; &BOUNDS is present but not followed by either one or two
          ;; names.
          (index
           (error "~@<Expected ~S START END or ~:*~S START but got ~:S.~@:>"
                  '&bounds (subseq lambda-list index)))
          ;; No &BOUNDS section.
          (t
           (let ((start (gensym "START"))
                 (end (gensym "END")))
             (values lambda-list
                     start
                     end
                     (list start end)))))
      (check-type start symbol)
      (check-type end symbol)
      (values lambda-list start end gensyms))))

(defun check-lambda-list (lambda-list spec
                          &key
                          (report-lambda-list lambda-list))
  (multiple-value-bind
        (required* optional* rest* keyword* allow-other-keys-p auxp keyp)
      (parse-ordinary-lambda-list lambda-list)
    (labels ((fail (expected actual)
               (let ((expected (ensure-list expected))
                     (actual   (ensure-list actual)))
                 (error "~@<Expected a lambda-list ~?, but ~:S ~?.~@:>"
                        (first expected) (rest expected)
                        report-lambda-list
                        (first actual) (rest actual))))
             (check-section (section expected actual)
               (typecase expected
                 ((eql nil)
                  (when actual
                    (fail (list "without ~A parameters" section)
                          (list "has ~A parameters" section))))
                 ((eql t)
                  (unless actual
                    (fail (list "with ~A parameters" section)
                          (list "has no ~A parameters" section))))
                 (integer
                  (unless (length= expected actual)
                    (fail (list "with ~D ~A parameter~:*~:P" expected section)
                          (list "has ~D ~A parameter~:*~:P"
                                (length actual) section))))))
             (check-binary (name expected actual)
               (when (member expected '(t nil))
                 (unless (eq expected (when actual t))
                   (fail (list "~:[without~;with~] ~A" expected name)
                         (list "~:[has no~;has~] ~A" actual name)))))
             (check-simple-spec (&key required optional rest
                                      keyword allow-other-keys aux key)
               (check-section "required"         required         required*)
               (check-section "optional"         optional         optional*)
               (check-binary  '&rest             rest             rest*)
               (check-section "keyword"          keyword          keyword*)
               (check-binary  '&allow-other-keys allow-other-keys allow-other-keys-p)
               (check-section "aux"              aux              auxp)
               (check-binary  '&key              key              keyp))
             (check-spec (spec)
               (typecase spec
                 ((cons (eql or))
                  (loop :with errors = ()
                     :for sub-spec :in (rest spec)
                     :do (handler-case
                             (progn
                               (check-spec sub-spec)
                               (return))
                           (error (condition)
                             (push condition errors)))
                     :finally (error "~@<~{~A~^~@:_~}~@:>" errors)))
                 (list
                  (apply #'check-simple-spec spec)))))
      (check-spec spec))))

(defun parse-defrule-options (options form)
  (let ((when (singleton-option 'defrule form :when t :default '(t . t)))
        (transform nil)
        (around nil)
        (error-report (singleton-option 'defrule form :error-report
                                        'rule-error-report :default t))
        (use-cache (singleton-option 'defrule form :use-cache
                                     'cache-policy :default :unless-trivial)))
    (dolist (option options)
      (with-current-source-form (option)
        (destructuring-ecase option
          ((:when expr &rest rest)
           (when rest
             (error "~@<Multiple expressions in a ~S:~@:_~2@T~S~@:>"
                    :when form))
           (funcall when (cons (cond
                                 ((not (constantp expr))
                                  `(lambda () ,expr))
                                 ((eval expr)
                                  t))
                               expr)))
          ((:constant value)
           (declare (ignore value))
           (push option transform))
          ((:text value)
           (when value
             (push option transform)))
          ((:identity value)
           (when value
             (push option transform)))
          ((:lambda lambda-list &body forms)
           (with-current-source-form (lambda-list option)
             (multiple-value-bind (lambda-list* start-var end-var ignore)
                 (parse-lambda-list-maybe-containing-&bounds lambda-list)
               (check-lambda-list lambda-list*
                                  '(or (:required 1) (:optional 1))
                                  :report-lambda-list lambda-list)
               (push (list :lambda lambda-list* start-var end-var ignore forms)
                     transform))))
          ((:function designator)
           (declare (ignore designator))
           (push option transform))
          ((:destructure lambda-list &body forms)
           (with-current-source-form (lambda-list option)
             (multiple-value-bind (lambda-list* start-var end-var ignore)
                 (parse-lambda-list-maybe-containing-&bounds lambda-list)
               (push (list :destructure lambda-list* start-var end-var ignore forms)
                     transform))))
          ((:around lambda-list &body forms)
           (with-current-source-form (lambda-list option)
             (multiple-value-bind (lambda-list* start end ignore)
                 (parse-lambda-list-maybe-containing-&bounds lambda-list)
               (check-lambda-list
                lambda-list* '() :report-lambda-list lambda-list)
               (setf around `(lambda (,start ,end transform)
                               (declare (ignore ,@ignore)
                                        (function transform))
                               (flet ((call-transform ()
                                        (funcall transform)))
                                 ,@forms))))))
          ((:use-cache value)
           (funcall use-cache value))
          ((:error-report behavior)
           (funcall error-report behavior)))))
    (values transform around (funcall when)
            (funcall error-report) (funcall use-cache))))

(defun expand-transforms (transforms)
  (let ((production-used-p t)
        (identityp t)
        (constantp nil)
        (textp nil))
    (labels
        ((make-transform-body (start end start-var end-var ignore body)
           (let* ((start-end-vars (list start-var end-var))
                  (other-ignore (set-difference ignore start-end-vars)))
             (multiple-value-bind (forms declarations) (parse-body body)
               `(,@(when other-ignore `((declare (ignore ,@other-ignore))))
                 ,@declarations
                 (let (,@(unless (member start-var ignore :test #'eq)
                           `((,start-var ,start)))
                       ,@(unless (member end-var ignore :test #'eq)
                           `((,end-var ,end))))
                   ,@forms)))))
         (process-option (options start end production)
           (destructuring-bind (&optional option &rest rest) options
             (unless option
               (return-from process-option production))
             (destructuring-ecase option
               ((:constant value)
                (setf production-used-p nil identityp nil constantp t)
                (process-option rest start end value))
               ((:identity value)
                (declare (ignore value))
                (process-option rest start end production))
               ((:text value)
                (setf textp (and value identityp)
                      identityp nil)
                (process-option rest start end `(text ,production)))
               ((:function designator)  ; TODO resolve-function?
                (setf identityp nil constantp nil)
                (process-option rest start end `(,designator ,production)))
               ((:lambda lambda-list start-var end-var ignore forms)
                (setf identityp nil constantp nil)
                (process-option
                 rest start end
                 `((lambda ,lambda-list
                     ,@(make-transform-body
                        start end start-var end-var ignore forms))
                   ,production)))
               ((:destructure lambda-list start-var end-var ignore forms)
                (setf identityp nil constantp nil)
                (process-option
                 rest start end
                 `(destructuring-bind ,lambda-list ,production
                    ,@(make-transform-body
                       start end start-var end-var ignore forms))))))))
      (with-gensyms (production start end)
        (let ((form (process-option (reverse transforms) start end production)))
          (values
           `(lambda (,production ,start ,end)
              (declare ,@(unless production-used-p `((ignore ,production)))
                       (ignorable ,start ,end))
              ,form)
           identityp
           constantp
           textp))))))
