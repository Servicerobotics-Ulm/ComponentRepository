;;;; rules-literals.lisp --- Rules for parsing common kinds of literals.
;;;;
;;;; Copyright (C) 2013-2019 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules)

#+esrap.grammar-class
(defgrammar #:parser.common-rules.literals
  (:documentation
   "This grammar contains rules for parsing number and string
    literals."))
#+esrap.grammar-class
(in-grammar #:parser.common-rules.literals)

;;; Boolean literals

(macrolet ((define-bool-syntax (name true-expression false-expression)
             (let ((literal-name (symbolicate '#:boolean-literal/ name))
                   (true-name    (symbolicate '#:true-literal/    name))
                   (false-name   (symbolicate '#:false-literal/   name)))
               `(progn
                  (defrule ,literal-name
                      (or ,true-name ,false-name))

                  (defrule ,true-name
                      ,true-expression
                    (:constant t))

                  (defrule ,false-name
                      ,false-expression
                    (:constant nil))))))

  (define-bool-syntax lower-case
    "true"
    "false")

  (define-bool-syntax capital-case
    "True"
    "False")

  (define-bool-syntax extended
    (or "true"  "t" "1")
    (or "false" "f" "0")))

;;; Number literals

(defrule sign
    (or #\- #\+)
  (:lambda (sign)
    (switch (sign :test #'string=)
      (#\- -1)
      (#\+  1))))

(defrule sign/?s
    (and sign whitespace*)
  (:function first))

(defun %parse-integer (radix text position end)
  (let+ (((&values value new-position)
          (parse-integer text :radix radix
                         :start position :end end :junk-allowed t)))
    (if value
        (values value new-position)
        (values nil   position))))

(macrolet
    ((define-integer-literal-rules (base-name base &key prefix)
       (let ((predicate-name    (symbolicate base-name '#:-digit-char?))
             (parse-name        (symbolicate '#:parse- base-name '#:-integer))
             (digits-rule-name  (symbolicate '#:integer-digits/ base-name))
             (no-sign-rule-name (symbolicate '#:integer-literal/ base-name '#:/no-sign))
             (main-rule-name    (symbolicate '#:integer-literal/ base-name))
             (prefix-rule-name  (symbolicate '#:integer-literal/ base-name '#:/prefix)))
         `(progn
            (defun ,predicate-name (character)
              (digit-char-p character ,base))

            (defun ,parse-name (text position end)
              (%parse-integer ,base text position end))

            (defrule ,digits-rule-name
                (+ (,predicate-name character))
              (:text t))

            (defrule ,no-sign-rule-name
                (and (! sign) (& (,predicate-name character)) #',parse-name)
              (:function third))

            (defrule ,main-rule-name
                (and (? sign/?s) ,no-sign-rule-name)
              (:destructure (sign magnitude)
                (* (or sign 1) magnitude)))

            ,@(when prefix
                `((defrule ,prefix-rule-name
                      (and (? sign/?s) ,prefix ,no-sign-rule-name)
                    (:destructure (sign prefix magnitude)
                      (declare (ignore prefix))
                      (* (or sign 1) magnitude)))))))))

  (define-integer-literal-rules #:binary       2)
  (define-integer-literal-rules #:octal        8 :prefix "0o")
  (define-integer-literal-rules #:decimal     10)
  (define-integer-literal-rules #:hexadecimal 16 :prefix "0x"))

(defrule integer-literal
    (or integer-literal/hexadecimal/prefix
        integer-literal/octal/prefix
        integer-literal/decimal))

(defrule float-decimals
    (and #\. (? integer-digits/decimal))
  (:function second)
  (:lambda (digits)
    (when digits
      (/ (parse-integer digits) (expt 10 (length digits))))))

(defrule float-scientific
    (and (~ #\e) integer-literal/decimal)
  (:function second)
  (:lambda (power)
    (expt 10 power)))

(defrule float-literal/rational
    (and (? sign/?s) (! sign)
         (or (and (? integer-literal/decimal) float-decimals     (? float-scientific))
             (and integer-literal/decimal     (? float-decimals) float-scientific)))
  (:destructure (sign nosign (digits decimals scientific))
    (declare (ignore nosign))
    (* (or sign 1) (+ (or digits 0) (or decimals 0)) (or scientific 1))))

(macrolet ((define-float-literal-rules (precision)
             (let* ((type               (symbolicate precision '#:-float))
                    (predicate-name     (symbolicate '#:in- type '#:-range))
                    (negative-name      (symbolicate '#:most-negative- type))
                    (positive-name      (symbolicate '#:most-positive- type))
                    (rule-name          (symbolicate precision '#:-float-literal))
                    (rational-rule-name (symbolicate rule-name '#:/rational)))
               `(progn
                  (defun ,predicate-name (number)
                    (if (<= ,negative-name number ,positive-name)
                        number
                        (values
                         nil
                         (format nil "~@<The value ~A is not within ~
                                      the bounds [~F, ~F] of the ~A ~
                                      type.~@:>"
                                 number ,negative-name ,positive-name
                                 ',type))))

                  (defrule ,rational-rule-name
                      (,predicate-name float-literal/rational))

                  (defrule ,rule-name
                      ,rational-rule-name
                    (:lambda (rational)
                      (coerce rational ',type)))))))

  (define-float-literal-rules single)
  (define-float-literal-rules double))

(defrule float-literal
    double-float-literal)

(defrule number-literal
    (or (and integer-literal/octal/prefix       (and))
        (and integer-literal/hexadecimal/prefix (and))
        (and integer-literal/decimal            (! #\.))
        (and float-literal                      (and)))
  (:function first))

;;; String literals

(defrule string-escape-character/hexadecimal
    (and #\x integer-literal/hexadecimal)
  (:function second)
  (:function code-char))

(defun <=-0-255 (number)
  (<= 0 number 255))

(defrule string-escape-character/octal
    (<=-0-255 integer-literal/octal)
  (:function code-char))

(defrule string-escape-character/character
    (or #\a #\b #\f #\n #\r #\t #\v)
  (:lambda (character)
    (switch (character :test #'string=)
      (#\a #\Bel)
      (#\b #\Backspace)
      (#\f #\Page)
      (#\n #\Newline)
      (#\r #\Return)
      (#\t #\Tab)
      (#\v #\Vt))))

(defrule string-escape-sequence
    (and #\\ (or #\\
                 string-escape-character/hexadecimal
                 string-escape-character/octal
                 string-escape-character/character)))

(macrolet ((define-string-literal-rule (name delimiter
                                        &key
                                        (escape? t))
             (let ((escape-name (symbolicate name '#:/escape)))
               `(progn
                  ,@(when escape?
                      `((defrule ,escape-name
                            (or (and #\\ ,delimiter)
                                string-escape-sequence)
                          (:function second))))

                  (defrule ,name
                      (and ,delimiter
                           (* ,(if escape?
                                   `(or ,escape-name
                                        (not (or ,delimiter #\\)))
                                   `(not ,delimiter)))
                           ,delimiter)
                    (:function second)
                    (:text t))))))

  (define-string-literal-rule string-literal/single-quotes   #\')
  (define-string-literal-rule string-literal/double-quotes   #\")

  ;; The following two may look a bit silly but are actually useful
  ;; for e.g. parsing Python code.
  (define-string-literal-rule string-literal/triple-quotes   "'''"
    :escape? nil)
  (define-string-literal-rule string-literal/sextuple-quotes "\"\"\""
    :escape? nil))
