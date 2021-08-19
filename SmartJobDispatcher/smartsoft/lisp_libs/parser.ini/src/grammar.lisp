;;;; grammar.lisp --- Grammar definition of the parser.ini system.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.ini)

#+esrap.grammar-class
(defgrammar #:parser.ini
  #+later (:use
   #:whitespace
   #:literals)
  (:documentation
   "Grammar for parsing \"ini-like\" configuration files. Some aspects
    of the grammar can be customized by binding special variables. "))
#+esrap.grammar-class
(in-grammar #:parser.ini)

(defrule comment/shell-style
    shell-style-comment
  (:constant nil)
  (:when (eql *comment-starter* #\#)))

(defrule comment/semicolon
    lisp-style-comment
  (:constant nil)
  (:when (eql *comment-starter* #\;)))

(defrule comment
    (or comment/shell-style comment/semicolon))

;; Quotes

(defrule escaped
    (and #\\ (or #\" #\\))
  (:function second))

(defrule quoted
    (and #\" (* (or escaped (not #\"))) #\")
  (:function second))

;; Names
;;
;; The specialized rules name-component-separator/. and
;; name-component-separator/: exist for performance reasons.

(defrule name-component-separator/.
    #\.
  (:when (eql *name-component-separator* #\.)))

(defrule |name-component-separator/:|
    #\:
  (:when (eql *name-component-separator* #\:)))

(eval-when (:compile-toplevel :load-toplevel :execute) ; avoid warning
  (defun parse-name-component-separator/expression (text start end)
    (esrap:parse *name-component-separator* text
                 #+esrap.grammar-class :grammar
                 #+esrap.grammar-class '#:parser.ini
                 :start start :end end
                 :junk-allowed t)))

(defrule name-component-separator/expression
    #'parse-name-component-separator/expression
  (:when (not (member *name-component-separator* '(#\. #\: nil)))))

(defrule name-component-separator
    (or name-component-separator/.
        |name-component-separator/:|
        name-component-separator/expression))

(defrule name-component
    (+ (or quoted
           (and #\[ name #\])
           (not (or comment
                    #\Newline
                    name-component-separator
                    #\[ #\]
                    (and whitespace* assignment-operator)))))
  (:text t))

(defrule name
    (and name-component
         (* (and name-component-separator name-component)))
  (:destructure (first rest)
    (cons first (mapcar #'second rest))))

;; Sections

(defrule section
    (and #\[ name #\])
  (:function second)
  (:lambda (name &bounds start end)
    (list :section (list name (cons start end)))))

;; Assignment variants
;;
;; The specialized rules assignment-operator/whitespace,
;; assignment-operator/= and assignment-operator/: exist for
;; performance reasons.

(defrule assignment-operator/whitespace whitespace+
  (:when (eq *assignment-operator* :whitespace)))

(defrule assignment-operator/= #\=
  (:when (eql *assignment-operator* #\=)))

(defrule |assignment-operator/:| #\:
  (:when (eql *assignment-operator* #\:)))

(eval-when (:compile-toplevel :load-toplevel :execute)
  (defun parse-assignment-operator/expression (text start end)
    (esrap:parse *assignment-operator* text
                 #+esrap.grammar-class :grammar
                 #+esrap.grammar-class '#:parser.ini
                 :start start :end end
                 :junk-allowed t)))

(defrule assignment-operator/expression
    #'parse-assignment-operator/expression
  (:when (not (member *assignment-operator* '(:whitespace #\. #\:)))))

(defrule assignment-operator
    (or assignment-operator/whitespace
        assignment-operator/=
        |assignment-operator/:|
        assignment-operator/expression))

;; Options

(defrule value-whitespace/all
    whitespace
  (:when (eq *value-terminating-whitespace-expression* :all)))

(defrule value-whitespace/newline
    #\Newline
  (:when (eql *value-terminating-whitespace-expression* #\Newline)))

(defrule value-whitespace
    (or value-whitespace/all value-whitespace/newline))

(defrule value
    (* (or quoted (not (or comment section value-whitespace))))
  (:text t))

(defrule option
    (and name
         (and whitespace*
              assignment-operator
              (* (and (! #\Newline) whitespace)))
         value)
  (:destructure (name operator value &bounds start end)
    (declare (ignore operator))
    (list :option (architecture.builder-protocol:node*
                      (:option
                       :name   name
                       :value  value
                       :bounds (cons start end))))))

;; Entry point

(defrule ini
    (* (or comment whitespace+ section option))
  (:lambda (value)
    ;; Add all options to their respective containing sections.
    (let+ ((sections      (make-array 0 :adjustable t :fill-pointer 0))
           (section-names (make-hash-table :test #'equal))
           ((&flet+ ensure-section ((name bounds))
              (aref sections
                    (ensure-gethash
                     name section-names
                     (vector-push-extend
                      (apply #'architecture.builder-protocol:make-node*
                             :section
                             :name name
                             (when bounds
                               (list :bounds bounds)))
                      sections)))))
           (current-section-info '(() nil)))
      (mapc (lambda+ ((&optional kind node))
              (ecase kind
                ((nil)) ; comment and whitespace
                (:section
                 (setf current-section-info node)
                 (when *include-empty-sections?*
                   (ensure-section current-section-info)))
                (:option
                 (let ((section (ensure-section current-section-info)))
                   (architecture.builder-protocol:relate*
                    :section-option section node)))))
            value)
      (map 'list (curry #'architecture.builder-protocol:finish-node* :section)
           sections))))
