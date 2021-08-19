;;;; rules-anchors.lisp --- Anchor rules.
;;;;
;;;; Copyright (C) 2016 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules)

;;; Anchor grammar

#+esrap.grammar-class
(defgrammar #:parser.common-rules.anchors
  (:documentation
   "This grammar contains rules that match if the input around the
    position in question has certain properties."))
#+esrap.grammar-class
(in-grammar #:parser.common-rules.anchors)

(defrule <beginning-of-input>
    (! (< 1 character))
  (:constant :beginning-of-input))

(defrule <end-of-input>
    (! character)
  (:constant :end-of-input))

(defrule <beginning-of-line>
    (or (& (< 1 #\Newline)) <beginning-of-input>)
  (:constant :beginning-of-line))

(defrule <end-of-line>
    (or (& (or #\Newline #\Page)) <end-of-input>)
  (:constant :end-of-line))

(defrule <same-line>
    (* (not (or #\Newline #\Page)))
  (:text t))
