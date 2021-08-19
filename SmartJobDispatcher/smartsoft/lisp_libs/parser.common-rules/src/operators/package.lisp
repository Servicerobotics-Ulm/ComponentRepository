;;;; package.lisp --- Package definition for parser.common-rules system.
;;;;
;;;; Copyright (C) 2012, 2013, 2015, 2016 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.common-rules.operators
  (:use
   #:cl
   #:alexandria
   #:let-plus

   #:esrap)

  (:import-from #:parser.common-rules
   #:skippable-rule-for-name)

  ;; Operator macros
  (:export
   #:define-unary-operator-rule
   #:define-binary-operator-rule
   #:define-ternary-operator-rule

   #:define-operator-rules)

  #+sbcl (:lock t)

  (:documentation
   "This package provides functions and macros for defining grammar
    rules for parsing unary and binary operators."))
