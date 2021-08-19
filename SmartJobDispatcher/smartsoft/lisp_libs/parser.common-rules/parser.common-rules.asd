;;;; parser.common-rules.asd --- System definition for the parser.common-rules system.
;;;;
;;;; Copyright (C) 2012-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(defsystem "parser.common-rules"
  :description "Provides common parsing rules that are useful in many grammars."
  :license     "MIT" ; see COPYING file for details.
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"

  :version     (:read-file-form "version-string.sexp")
  :depends-on  ("alexandria"
                "split-sequence" ; for rules-comments.lisp
                (:version "let-plus" "0.2")
                (:version "esrap"    "0.16"))

  :components  ((:module     "src"
                 :serial     t
                 :components ((:file       "package")

                              (:file       "rules-anchors")
                              (:file       "rules-whitespace")
                              (:file       "rules-comments")
                              (:file       "rules-literals")

                              (:file       "macros-tokenization"))))

  :in-order-to ((test-op (test-op "parser.common-rules/test"))))

(defsystem "parser.common-rules/test"
  :description "Tests for the parser.common-rules system."
  :license     "MIT" ; see COPYING file for details.
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"

  :version     (:read-file-form "version-string.sexp")
  :depends-on  ("alexandria"
                (:version "let-plus"            "0.2")

                (:version "fiveam"              "1.4")

                (:version "parser.common-rules" (:read-file-form "version-string.sexp")))

  :components  ((:module     "test"
                 :serial     t
                 :components ((:file       "package")

                              (:file       "rules-anchors")
                              (:file       "rules-comments")
                              (:file       "rules-literals")

                              (:file       "macros-tokenization"))))

  :perform (test-op (operation component)
             (uiop:symbol-call '#:parser.common-rules.test '#:run-tests)))
