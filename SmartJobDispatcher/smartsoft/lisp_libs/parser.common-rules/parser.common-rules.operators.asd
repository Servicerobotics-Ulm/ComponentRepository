;;;; parser.common-rules.operators.asd --- System definition for the parser.common-rules.operators system.
;;;;
;;;; Copyright (C) 2012-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(defsystem "parser.common-rules.operators"
  :description "Provides macros for defining grammar rules for infix operators."
  :license     "MIT" ; see COPYING file for details.
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"

  :version     (:read-file-form "version-string.sexp")
  :depends-on  ("alexandria"
                (:version "let-plus"                      "0.2")
                (:version "esrap"                         "0.9")
                (:version "architecture.builder-protocol" "0.1")

                (:version "parser.common-rules"           (:read-file-form "version-string.sexp")))

  :components  ((:module     "src/operators"
                 :serial     t
                 :components ((:file       "package")

                              (:file       "macros"))))

  :in-order-to ((test-op (test-op "parser.common-rules.operators/test"))))

(defsystem "parser.common-rules.operators/test"
  :description "Tests for the parser.common-rules.operators system."
  :license     "MIT" ; see COPYING file for details.
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"

  :version     (:read-file-form "version-string.sexp")
  :depends-on  ((:version "fiveam"                        "1.4")

                (:version "parser.common-rules.operators" (:read-file-form "version-string.sexp"))

                (:version "parser.common-rules/test"      (:read-file-form "version-string.sexp")))

  :components  ((:module     "test/operators"
                 :serial     t
                 :components ((:file       "package")

                              (:file       "macros"))))

  :perform (test-op (operation component)
             (uiop:symbol-call '#:parser.common-rules.operators.test '#:run-tests)))
