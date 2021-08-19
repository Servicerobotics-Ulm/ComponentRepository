;;;; package.lisp --- Package definition for tests of the parser.infix-expressions system.
;;;;
;;;; Copyright (C) 2013, 2016 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.common-rules.operators.test
  (:use
   #:cl
   #:alexandria
   #:let-plus

   #:fiveam

   #:parser.common-rules.operators)

  (:import-from #:parser.common-rules.test
   #:parses-are)

  (:export
   #:run-tests)

  (:documentation
   "This package contains tests of the parser.common-rules.operators
    system."))

(cl:in-package #:parser.common-rules.operators.test)

;;; Root test suite

(def-suite :parser.common-rules.operators)

(defun run-tests ()
  (run! :parser.common-rules.operators))
