;;;; package.lisp --- Package definition for tests of the parser.common-rules system.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.common-rules.test
  (:use
   #:cl
   #:alexandria
   #:let-plus

   #:fiveam

   #:parser.common-rules)

  ;; Test utilities
  (:export
   #:parses-are
   #:define-rule-test)

  ;; Root test suite
  (:export
   #:run-tests)

  (:documentation
   "This package contains tests of the parser.common-rules system."))

(cl:in-package #:parser.common-rules.test)

;;; Test utilities

(defmacro parses-are ((rule) &body cases)
  (let+ (((&flet+ process-case ((input-and-args
                                 expected-production
                                 &optional
                                 (expected-position (if expected-production
                                                        nil
                                                        0))
                                 (expected-success  (if expected-production
                                                        t
                                                        nil))))
            (let+ (((input &rest args) (ensure-list input-and-args)))
              `(is (equal (list ,expected-production
                                ,expected-position
                                ,expected-success)
                          (parse ,input ,@args :junk-allowed t)))))))
    `(flet ((parse (input &rest args)
              (let+ ((input (format nil input))
                     ((&values production position success)
                      (apply #'esrap:parse ',rule input args)))
                (list production position success))))
       (declare (ignorable #'parse)
                (notinline esrap:parse))
       ,@(mapcar #'process-case cases))))

(defmacro define-rule-test (rule &body cases)
  (let ((test-name (symbolicate '#:rule. rule)))
    `(test ,test-name
       ,(format nil "Smoke test for the `~(~A~)' rule." rule)
       (parses-are (,rule) ,@cases))))

;;; Root test suite

(def-suite :parser.common-rules
  :description
  "Root test suite of the parser.common-rules system.")

(defun run-tests ()
  (run! :parser.common-rules))
