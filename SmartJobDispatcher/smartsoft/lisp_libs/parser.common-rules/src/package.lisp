;;;; package.lisp --- Package definition for parser.common-rules system.
;;;;
;;;; Copyright (C) 2012-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.common-rules
  (:use
   #:cl
   #:alexandria
   #:let-plus

   #:esrap)

  ;; Anchor rules
  (:export
   #:<beginning-of-input> #:<end-of-input>
   #:<beginning-of-line>  #:<end-of-line>

   #:<same-line>)

  ;; Whitespace rules
  (:export
   #:whitespace/not-newline #:whitespace/not-newline?
   #:whitespace             #:whitespace?             #:whitespace+ #:whitespace*)

  ;; Comment rules
  (:export
   #:c-style-comment/rest-of-line #:c-style-comment/rest-of-line/trimmed
   #:c-style-comment/delimited    #:c-style-comment/delimited/trimmed

   #:shell-style-comment          #:shell-style-comment/trimmed

   #:lisp-style-comment           #:lisp-style-comment/trimmed)

  ;; Literal rules
  (:export
   #:boolean-literal/lower-case
   #:boolean-literal/capital-case
   #:boolean-literal/extended

   #:integer-literal/binary
   #:integer-literal/binary/no-sign

   #:integer-literal/octal
   #:integer-literal/octal/prefix
   #:integer-literal/octal/no-sign

   #:integer-literal/decimal
   #:integer-literal/decimal/no-sign

   #:integer-literal/hexadecimal
   #:integer-literal/hexadecimal/prefix
   #:integer-literal/hexadecimal/no-sign

   #:integer-literal

   #:single-float-literal/rational #:single-float-literal
   #:double-float-literal/rational #:double-float-literal
   #:float-literal/rational        #:float-literal

   #:number-literal

   #:string-literal/single-quotes
   #:string-literal/double-quotes
   #:string-literal/triple-quotes
   #:string-literal/sextuple-quotes)

  ;; Tokenization macros
  (:export
   #:defrule/s)

  #+sbcl (:lock t)

  (:documentation
   "This package contains parsing rules and macros for common parsing
    tasks that are hopefully useful in many grammars."))
