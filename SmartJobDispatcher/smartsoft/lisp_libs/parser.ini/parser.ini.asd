;;;; parser.ini.asd --- System definition for the parser.ini system.
;;;;
;;;; Copyright (C) 2012-2017 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(defsystem :parser.ini
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :version     (:read-file-form "version-string.sexp")
  :license     "LLGPLv3" ; see COPYING file for details.
  :description "Provides parsing of Ini expressions."
  :depends-on  (:alexandria
                (:version :let-plus                      "0.2")
                (:version :more-conditions               "0.1.0")

                (:version :esrap                         "0.15")
                (:version :architecture.builder-protocol "0.1")
                (:version :parser.common-rules           "0.1"))
  :encoding    :utf-8
  :components  ((:module     "src"
                 :serial     t
                 :components ((:file       "package")
                              (:file       "conditions")
                              (:file       "variables")
                              (:file       "protocol")
                              (:file       "grammar")))

                (:static-file "README.org")

                (:module     "examples"
                 :components ((:static-file "etc.lisp"))))
  :in-order-to ((test-op (test-op :parser.ini/test))))

(defsystem :parser.ini/test
  :author      "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :maintainer  "Jan Moringen <jmoringe@techfak.uni-bielefeld.de>"
  :version     (:read-file-form "version-string.sexp")
  :license     "LLGPLv3" ; see COPYING file for details.
  :description "Provides parsing of Ini expressions."
  :depends-on  (:alexandria
                (:version :let-plus   "0.2")
                (:version :fiveam     "1.3")

                (:version :parser.ini (:read-file-form "version-string.sexp")))
  :encoding    :utf-8
  :components  ((:module     "test"
                 :serial     t
                 :components ((:file       "package")
                              (:file       "protocol")
                              (:file       "grammar")))))

(defmethod perform ((op     test-op)
                    (system (eql (find-system :parser.ini/test))))
  (uiop:symbol-call '#:parser.ini.test '#:run-tests))
