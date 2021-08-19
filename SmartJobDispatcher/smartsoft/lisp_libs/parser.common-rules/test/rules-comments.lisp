;;;; rules-comments.lisp --- Tests for comment-related rules.
;;;;
;;;; Copyright (C) 2016, 2017, 2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules.test)

(def-suite* :parser.common-rules.comment-rules
  :in :parser.common-rules)

;;; C-style comments

(define-rule-test c-style-comment/rest-of-line
  ;; Some matching inputs
  ("//"          "")
  ("// foo"      " foo")
  ("// foo~%bar" " foo" 6)

  ;; Some non-matching inputs.
  (""            nil)
  ("a"           nil)
  ("/"           nil)
  ("/a/"         nil))

(define-rule-test c-style-comment/rest-of-line/trimmed
  ;; Only testing matching inputs.
  ("//"         "")
  ("// "        " ")
  ("// foo"     " foo")
  ("// foo~%"   " foo"  6)
  ("// foo "    " foo ")
  ("// foo ~%"  " foo " 7)

  ("///"        "")
  ("/// "       " ")
  ("/// foo"    " foo")
  ("/// foo~%"  " foo"  7)
  ("/// foo "   " foo ")
  ("/// foo ~%" " foo " 8)

  ("// //"      " //"))

(define-rule-test c-style-comment/delimited
  ;; Some matching inputs.
  ("/**/"        "")
  ("/* foo*/"    " foo")
  ("/* // */"    " // ")
  ("/* /* */"    " /* ")

  ;; Some non-matching inputs.
  (""            nil)
  ("a"           nil)
  ("/"           nil)
  ("/*"          nil)
  ("/**"         nil)
  ("//"          nil))

(define-rule-test c-style-comment/delimited/trimmed
  ;; Only testing matching inputs.
  ("/**/"                    "")
  ("/*~%*/"                  "")
  ("/*~%~%*/"                "")
  ("/*foo*/"                 "foo")
  ("/*~%foo*/"               "foo")
  ("/*foo~%*/"               "foo")
  ("/*~%foo~%*/"             "foo")
  ("/*~%foo~%bar*/"          (format nil "foo~%bar"))
  ("/* foo~% * bar~% */"     (format nil " foo~%bar"))
  ("/*~% * foo~% * bar~% */" (format nil "foo~%bar"))
  ("/** foo~% * bar~% */"    (format nil "* foo~%bar")))

;;; Shell-style comments

(define-rule-test shell-style-comment
  ;; Some matching inputs.
  ("#"          "")
  ("# foo"      " foo")
  ("# foo~%bar" " foo" 5)

  ;; Some non-matching inputs.
  (""           nil)
  ("a"          nil))

(define-rule-test shell-style-comment/trimmed
  ;; Only testing matching inputs.
  ("#"         "")
  ("# "        " ")
  ("# foo"     " foo")
  ("# foo~%"   " foo"  5)
  ("# foo "    " foo ")
  ("# foo ~%"  " foo " 6)

  ("##"        "")
  ("## "       " ")
  ("## foo"    " foo")
  ("## foo~%"  " foo"  6)
  ("## foo "   " foo ")
  ("## foo ~%" " foo " 7))

;;; Lisp-style comments

(define-rule-test lisp-style-comment
  ;; Some matching inputs.
  (";"          "")
  ("; foo"      " foo")
  ("; foo~%bar" " foo" 5)

  ;; Some non-matching inputs.
  (""           nil)
  ("a"          nil))

(define-rule-test lisp-style-comment/trimmed
  ;; Only testing matching inputs.
  (";"         "")
  ("; "        " ")
  ("; foo"     " foo")
  ("; foo~%"   " foo"  5)
  ("; foo "    " foo ")
  ("; foo ~%"  " foo " 6)

  (";;"        "")
  (";; "       " ")
  (";; foo"    " foo")
  (";; foo~%"  " foo"  6)
  (";; foo "   " foo ")
  (";; foo ~%" " foo " 7))
