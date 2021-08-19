;;;; rules-comments.lisp --- Rules for different kinds of comments.
;;;;
;;;; Copyright (C) 2013, 2014, 2015, 2016, 2017 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules)

#+esrap.grammar-class
(defgrammar #:parser.common-rules.comments
  (:use #:parser.common-rules.whitespace)
  (:documentation
   "This grammar contains rules for parsing different kinds of
    comments."))
#+esrap.grammar-class
(in-grammar #:parser.common-rules.comments)

;;; C-style comments

(defrule c-style-comment/rest-of-line
    (and "//" <same-line>)
  (:function second))

(defrule c-style-comment/rest-of-line/trimmed
    c-style-comment/rest-of-line
  (:lambda (content)
    (string-left-trim '(#\/) content)))

(defrule c-style-comment/delimited
    (and "/*" (* (not "*/")) "*/")
  (:function second)
  (:text t))

(defrule c-style-comment/delimited/trimmed
    c-style-comment/delimited
  (:lambda (content)
    (trim-common-prefix content :prefix-characters '(#\Space #\*))))

(defrule c-style-comment
    (or c-style-comment/rest-of-line c-style-comment/delimited))

;;; Shell-style comments

(defrule shell-style-comment
    (and #\# <same-line>)
  (:function second))

(defrule shell-style-comment/trimmed
    shell-style-comment
  (:lambda (content)
    (string-left-trim '(#\#) content)))

;;; Lisp-style comments

(defrule lisp-style-comment
    (and #\; <same-line>)
  (:function second))

(defrule lisp-style-comment/trimmed
    lisp-style-comment
  (:lambda (content)
    (string-left-trim '(#\;) content)))

;;; Utility functions

(defgeneric trim-common-prefix (thing &key prefix-characters)
  (:method ((thing cons) &key (prefix-characters '(#\Space)))
    (let+ (((first &rest rest) thing)
           ((&flet prefix-length (line)
              (or (position-if-not (rcurry #'member prefix-characters) line)
                  most-positive-fixnum)))
           (prefix-length (reduce #'min rest
                                  :key           #'prefix-length
                                  :initial-value most-positive-fixnum))
           (trimmed       (mapcar (lambda (line)
                                    (subseq line (min prefix-length
                                                      (length line))))
                                  rest))
           (trimmed       (if (emptyp (lastcar trimmed))
                              (butlast trimmed)
                              trimmed)))
      (if (emptyp first)
          trimmed
          (list* first trimmed))))
  (:method ((thing string) &key (prefix-characters '(#\Space)))
    (let* ((lines   (split-sequence:split-sequence #\Newline thing))
           (trimmed (trim-common-prefix
                     lines :prefix-characters prefix-characters)))
      (format nil "~{~A~^~%~}" trimmed))))
