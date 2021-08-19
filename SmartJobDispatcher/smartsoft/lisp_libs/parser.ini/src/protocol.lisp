;;;; protocol.lisp --- Protocol provided by the parser.ini system.
;;;;
;;;; Copyright (C) 2012-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.ini)

;;; Parser protocol

(defgeneric parse (source builder &key start end junk-allowed)
  (:documentation
   "Parse the content of SOURCE as \"ini-like\" configuration options,
    construct a parse result using BUILDER and return it.

    START and END can be used to restrict parsing to a sub-sequence of
    SOURCE.

    JUNK-ALLOWED controls whether an error is signaled when a
    successful parse does not consume the entire input in SOURCE (or
    the sub-sequence delimited by START and END).

    Signal a `ini-parse-error' when errors are encountered."))

;; Default behavior

(define-condition-translating-method parse ((source t) (builder t)
                                            &key start end junk-allowed)
  ;; When an `esrap-error' is signaled, the innermost `parse' method
  ;; is specialized on `string'. It is therefore OK to assign SOURCE
  ;; to the source slot of the condition.
  ((esrap:esrap-error ini-parse-error
    :var condition)
   :source   source
   :location (when (esrap:esrap-error-position condition)
               (cons (esrap:esrap-error-position condition) nil)))
  ((error ini-parse-error)
   :source source))

(defmethod parse ((source t) (builder t)
                  &key start end junk-allowed)
  (declare (ignore start end junk-allowed))
  (error "~@<Cannot parse source ~A with builder ~A.~@:>"
         source builder))

(defmethod parse ((source string) (builder t)
                  &key (start 0) end junk-allowed)
  (architecture.builder-protocol:with-builder (builder)
    (esrap:parse 'ini source
                 :start start :end end :junk-allowed junk-allowed
                 #+esrap.grammar-class :grammar
                 #+esrap.grammar-class '#:parser.ini)))

(defmethod parse ((source stream) (builder t)
                  &key (start 0) end junk-allowed)
  (parse (read-stream-content-into-string source) builder
         :start start :end end :junk-allowed junk-allowed))

(defmethod parse ((source pathname) (builder t)
                  &key (start 0) end junk-allowed)
  (parse (read-file-into-string source) builder
         :start start :end end :junk-allowed junk-allowed))
