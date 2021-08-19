;;;; grammar.lisp --- Unit tests for the ini grammar.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.ini.test)

(in-suite parser.ini)

(test grammar.smoke
  "Smoke test for the ini grammar."

  (mapc
   (lambda+ ((input expected &rest args))
     (let+ ((input (format nil input))
            ((&flet do-it ()
               (apply #'parse input 'list args))))
       (case expected
         (ini-parse-error (signals ini-parse-error (do-it)))
         (t               (is (equal expected (do-it)))))))

   '((""             nil) ; Empty
     ("#"            nil) ; Empty comment
     ("# # comment"  nil) ; Comment in comment
     ("# no content" nil) ; Comment without content

     ;; Empty value
     ("foo="
      ((:section
        (:section-option (((:option () :name ("foo") :value "" :bounds (0 . 4)))
                          . ()))
        :name ())))
     ("foo=~@
       bar=1"
      ((:section
        (:section-option (((:option () :name ("foo") :value "" :bounds (0 . 4)) . ())
                          ((:option () :name ("bar") :value "1" :bounds (5 . 10) . ()))))
        :name ())))

     ;; Junk not parsed as name of option
     ("bar: baz
       fez: whoop=>1"
      nil
      :junk-allowed t)

     ;; Comment after value
     ("foo = 1 # comment"
      ((:section
        (:section-option (((:option () :name ("foo") :value "1" :bounds (0 . 7))
                           . ())))
        :name ())))

     ;; Repeated section
     ("[foo] bar = 1 [foo] baz = 2"
      ((:section
        (:section-option
         (((:option () :name ("bar") :value "1" :bounds (6 . 13)) . ())
          ((:option () :name ("baz") :value "2" :bounds (20 . 27)) . ())))
        :name   ("foo")
        :bounds (0 . 5))))

     ;; [] in section name
     ("[foo[bar]] baz=2"
      ((:section
        (:section-option
         (((:option () :name ("baz") :value "2" :bounds (11 . 16)))))
        :name   ("foo[bar]")
        :bounds (0 . 10))))

     ;; whitespace in section name
     ("[foo bar] baz=2"
      ((:section
        (:section-option
         (((:option () :name ("baz") :value "2" :bounds (10 . 15)))))
        :name   ("foo bar")
        :bounds (0 . 9))))

     ;; Fancy escaping in names
     ("[foo.\"b\\az.fe\\\"z\"] frob.bar\".whoop.di do\"o = 1"
      ((:section
        (:section-option
         (((:option () :name ("frob" "bar.whoop.di doo") :value "1"
                    :bounds (19 . 46))
           . ())))
        :name   ("foo" "b\\az.fe\"z")
        :bounds (0 . 18))))

     ;; whitespace in option name
     ("foo bar = "
      ((:section
        (:section-option
         (((:option () :name ("foo bar") :value "" :bounds (0 . 10)))))
        :name ())))

     ;; Empty value
     ("bar = "
      ((:section
        (:section-option
         (((:option () :name ("bar") :value "" :bounds (0 . 6)))))
        :name ())))

     ;; Fancy escaping in values
     ("bar = \"wh\\\"o\\\\op \"di\" doo\""
      ((:section
        (:section-option
         (((:option () :name ("bar") :value "wh\"o\\op di doo" :bounds (0 . 26))
           . ())))
        :name ())))

     ;; Multiple lines
     ("a.b=1~@
       a=2~@
       # comment~@
       [a]~@
       c = 3~@
       [d]~@
       e = 4"
      ((:section
        (:section-option
         (((:option () :name ("a" "b") :value "1" :bounds (0 . 5)) . ())
          ((:option () :name ("a") :value "2" :bounds (6 . 9)) . ())))
        :name ())
       (:section
        (:section-option
         (((:option () :name ("c") :value "3" :bounds (24 . 29)) . ())))
        :name ("a") :bounds (20 . 23))
       (:section
        (:section-option
         (((:option () :name ("e") :value "4" :bounds (34 . 39)) . ())))
        :name ("d") :bounds (30 . 33)))))))

(test grammar.comment-starter
  "Tests controlling comment starter via special variable."

  (mapc
   (lambda+ ((comment-starter input expected))
     (let+ (((&flet do-it ()
               (let ((*comment-starter* comment-starter))
                 (parse input 'list)))))
       (case expected
         (ini-parse-error (signals ini-parse-error (do-it)))
         (t               (is (equal expected (do-it)))))))

   '(;; Disallow comments
     (nil "; comment" ini-parse-error)
     (nil "# comment" ini-parse-error)

     ;; ";" starts comments
     (#\; "; comment" ())
     (#\; "# comment" ini-parse-error)

     ;; "#" starts comments
     (#\# "; comment" ini-parse-error)
     (#\# "# comment" ()))))

(test grammar.name-component-separator
  "Tests controlling name component separator via special variable."

  (mapc
   (lambda+ ((separator input expected))
     (let+ (((&flet do-it ()
               (let ((*name-component-separator* separator))
                 (parse input 'list)))))
       (case expected
         (ini-parse-error (signals ini-parse-error (do-it)))
         (t               (is (equal expected (do-it)))))))

   '((nil "[foo.bar] baz.fez = 1"
      ((:section
        (:section-option
         (((:option () :name ("baz.fez") :value "1" :bounds (10 . 21)) . ())))
        :name ("foo.bar") :bounds (0 . 9))))

     (#\. "[foo.bar] baz.fez = 1"
      ((:section
        (:section-option
         (((:option () :name ("baz" "fez") :value "1" :bounds (10 . 21)) . ())))
        :name ("foo" "bar") :bounds (0 . 9))))

     (#\: "[foo:bar] baz.fez = 1"
      ((:section
        (:section-option
         (((:option () :name ("baz.fez") :value "1" :bounds (10 . 21)) . ())))
        :name ("foo" "bar") :bounds (0 . 9)))))))

(test grammar.include-empty-sections
  "Tests controlling the behavior for empty sections via special
   variable."

  (mapc
   (lambda+ ((include? input expected))
     (let ((*include-empty-sections?* include?))
       (is (equal expected (parse input 'list)))))

   '((nil "[foo]"
      ())

     (nil "[foo] bar = 1 "
      ((:section
        (:section-option
         (((:option () :name ("bar") :value "1" :bounds (6 . 13)) . ())))
        :name ("foo") :bounds (0 . 5))))

     (t   "[foo]"
      ((:section () :name ("foo") :bounds (0 . 5))))

     (t   "[foo] bar = 1 "
      ((:section
        (:section-option
         (((:option () :name ("bar") :value "1" :bounds (6 . 13)) . ())))
        :name ("foo") :bounds (0 . 5)))))))
