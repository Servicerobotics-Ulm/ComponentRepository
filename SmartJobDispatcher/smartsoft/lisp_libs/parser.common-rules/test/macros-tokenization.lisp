;;;; macros-tokenization.lisp --- Tests for tokenization macros.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules.test)

(def-suite* :parser.common-rules.tokenization-macros
  :in :parser.common-rules)

;; Mock "skippable" support rules.
(esrap:defrule defrule/s.test.skippable?
    (* #\Space)
  (:text t))
(esrap:defrule defrule/s.test.skippable
    (and #\Space defrule/s.test.skippable?)
  (:text t))

(test macro.defrule/s.smoke
  "Smoke test for `defrule/s' macro."

  ;; Main rules.
  (defrule/s (defrule/s.test
              :skippable-expression  defrule/s.test.skippable
              :skippable?-expression defrule/s.test.skippable?)
    "foo")

  ;; Test variants.
  (parses-are (defrule/s.test)
    ("foo"  "foo" nil)
    ("foo " "foo" 3))

  (parses-are (defrule/s.test/s)
    ("foo"  nil)
    ("foo " "foo" nil))

  (parses-are (defrule/s.test/?s)
    ("foo"  "foo" nil)
    ("foo " "foo" nil)))

(test macro.defrule/s.no-skippable-rule
  "Test error signaled by `defrule/s' in case the package has no
   skippable rule."

  (signals error
    (macroexpand '(defrule/s (defrule/s.test)
                    "foo")))

  (signals error
    (macroexpand '(defrule/s (defrule/s.test
                              :skippable-expression "")
                    "foo")))

  (signals error
    (macroexpand '(defrule/s (defrule/s.test
                              :skippable?-expression "")
                    "foo")))

  (finishes
    (macroexpand '(defrule/s (defrule/s.test
                              :skippable-expression  ""
                              :skippable?-expression "")
                    "foo"))))
