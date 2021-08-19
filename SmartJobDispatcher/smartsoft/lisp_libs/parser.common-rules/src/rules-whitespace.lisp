;;;; rules-whitespace.lisp --- Common whitespace-related rules.
;;;;
;;;; Copyright (C) 2013, 2014, 2015, 2016, 2017 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules)

;;; Whitespace grammar

#+esrap.grammar-class
(defgrammar #:parser.common-rules.whitespace
  (:documentation
   "This grammar contains rules for handling combinations of
    whitespace such as space, tab and newline."))
#+esrap.grammar-class
(in-grammar #:parser.common-rules.whitespace)

(defrule whitespace/not-newline
    (+ (or #\Space #\Tab))
  (:constant nil))

(defrule whitespace/not-newline?
    (? whitespace/not-newline)
  (:error-report nil))

(defrule whitespace
    (or #\Space #\Tab #\Newline #\Page)
  (:constant nil))

(defrule whitespace?
    (? whitespace)
  (:error-report nil))

(defrule whitespace+
    (and whitespace whitespace*)
  (:constant nil))

(defrule whitespace*
    (* whitespace)
  (:error-report nil)
  (:constant nil))
