;;;; package.lisp --- Package definition for parser.ini system.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:defpackage #:parser.ini
  (:use
   #:cl
   #:alexandria
   #:let-plus
   #:more-conditions

   #:parser.common-rules)

  (:import-from #:esrap
   #+esrap.grammar-class #:defgrammar
   #+esrap.grammar-class #:in-grammar
   #:defrule #:&bounds
   #:? #:!
   #:text)

  ;; Conditions
  (:export
   #:ini-parse-error

   #:ini-parse-error-source
   #:ini-parse-error-location)

  ;; Variables
  (:export
   #:*comment-starter*
   #:*name-component-separator*
   #:*assignment-operator*
   #:*value-terminating-whitespace-expression*
   #:*include-empty-sections?*)

  ;; Parser protocol
  (:export
   #:parse)

  (:documentation
   "This package provides the main entry point

    parse SOURCE BUILDER                [generic function]

      Parse the content of SOURCE as \"ini-like\" configuration
      options, construct a parse result using BUILDER and return it.

    The builder protocol consists of

    architecture.builder-protocol:make-node BUILDER KIND &rest ARGS [generic function]

      Create objects representing sections and options with
      KIND :section and :option respectively.

    architecture.builder-protocol:relate BUILDER RELATION LEFT RIGHT &rest ARGS [generic function]

      Attach options to their containing sections with
      relation :section-option.

    Parsing may signal

    ini-parse-error                     [condition]

    Syntactic variants can be controlled by binding the special
    variables

    *comment-starter*                   [special variable]

      Controls which character initiates a comment. Defaults to \"#\".

    *assignment-expression*             [special variable]

      Controls assignment expression. Defaults to \"=\".

    *value-terminating-whitespace-expression* [special variable]

      Controls which whitespace terminates option values. By default,
      all whitespace terminates option values."))
