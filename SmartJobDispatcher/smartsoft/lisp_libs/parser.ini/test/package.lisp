;;;; package.lisp --- Package definition for unit tests of the parser.ini system.
;;;;
;;;; Copyright (C) 2013, 2015, 2016 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.ini.test
  (:use
   #:cl
   #:let-plus

   #:fiveam

   #:parser.ini)

  (:export
   #:run-tests)

  (:documentation
   "This package contains unit tests of the parser.ini system."))

(cl:in-package #:parser.ini.test)

(def-suite parser.ini
  :description
  "Root unit test suite for the parser.ini system.")

(defun run-tests ()
  (run! 'parser.ini))
