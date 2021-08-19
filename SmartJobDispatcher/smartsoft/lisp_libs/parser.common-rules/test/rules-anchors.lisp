;;;; rules-anchors.lisp --- Tests for anchor rules.
;;;;
;;;; Copyright (C) 2016, 2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules.test)

(def-suite* :parser.common-rules.anchor-rules
  :in :parser.common-rules)

(define-rule-test <beginning-of-input>
  (""             :beginning-of-input)

  (("a" :start 1) nil                 1))

(define-rule-test <end-of-input>
  (""  :end-of-input)

  ("a" nil))

(define-rule-test <beginning-of-line>
  (""             :beginning-of-line)

  (("a" :start 1) nil                1))

(define-rule-test <end-of-line>
  (""   :end-of-line)
  ("~%" :end-of-line 0 t)

  ("a"  nil))

(define-rule-test <same-line>
  ("foo~%bar" "foo" 3 t))
