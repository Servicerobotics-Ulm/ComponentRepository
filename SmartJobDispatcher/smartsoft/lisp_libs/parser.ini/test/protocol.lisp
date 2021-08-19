;;;; protocol.lisp --- Unit tests for the protocol of the parser.ini system.
;;;;
;;;; Copyright (C) 2013, 2016 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.ini.test)

(in-suite parser.ini)

(test parse.smoke
  "Smoke test for the generic function `parse'."

  (mapc
   (lambda+ ((input builder args expected))
     (let+ (((&flet do-it () (apply #'parse input builder args))))
       (case expected
         (ini-parse-error (signals ini-parse-error (do-it)))
         (t               (is (equal expected (do-it)))))))

   `(;; Some invalid cases
     (:no-such-method   list             ()         ini-parse-error)
     (,#P"no-such-file" list             ()         ini-parse-error)
     ("foo = bar"       :no-such-builder ()         ini-parse-error)

     ;; These are valid
     ("foo = bar"       list             ()
      ((:section
        (:section-option (((:option
                            ()
                            :name ("foo") :value "bar" :bounds (0 . 9)))))
        :name nil)))

     ("= foo = bar"     list             (:start 2)
      ((:section
        (:section-option (((:option
                            ()
                            :name ("foo") :value "bar" :bounds (2 . 11)))))
        :name nil)))

     ("= "              list             (:junk-allowed t)
      ()))))
