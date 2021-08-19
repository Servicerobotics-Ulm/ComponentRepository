(cl:require :parser.ini)

(cl:in-package #:cl-user)

;; Parse options in /etc/colord.conf, building the result with the
;; `list' builder.
;;
;; This file uses:
;; * "#" for whole-line comments
;; * "=" for assignments
;; * all whitespace /seems/ to delimit values
(let ((parser.ini:*assignment-operator*                     #\=)
      (parser.ini:*value-terminating-whitespace-expression* :all))
  (parser.ini:parse #P"/etc/colord.conf" 'list))

;; Parse options in /etc/debconf, building the result with the `list'
;; builder.
;;
;; This file uses:
;; * "#" for whole-line comments
;; * ":" for assignments
;; * newlines to delimit values
(let ((parser.ini:*assignment-operator*                     #\:)
      (parser.ini:*value-terminating-whitespace-expression* #\Newline))
  (parser.ini:parse #P"/etc/debconf.conf" 'list))
