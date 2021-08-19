;;;; macros-tokenization.lisp --- Macros that aid with common tokenization issues.
;;;;
;;;; Copyright (C) 2012-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules)

;;; Utilities

(defun skippable-rule-for-name (skippable-name name)
  ;; The initial idea was using (symbol-package name) but that doesn't
  ;; work when the symbol naming the rule is from a different package
  ;; than the current package (at `defrule/s' expansion time).
  (let ((package *package*))
    (or (find-symbol (string skippable-name))
        (error "~@<Could not find ~A rule in package ~A for name ~
                ~S.~@:>"
               skippable-name package name))))

;;; Macros

(defmacro defrule/s (name-and-options expression &body options)
  "Like `esrap:defule' but define additional rules named NAME/s and
   NAME/?s which respectively require/allow EXPRESSION to be followed
   by skippable input (e.g. whitespace).

   NAME-AND-OPTIONS can be either just a rule name or a list of the
   form

     (NAME &key
           SKIPPABLE-EXPRESSION  S?
           SKIPPABLE?-EXPRESSION ?S?
           DEFINER)

   where SKIPPABLE-EXPRESSION and SKIPPABLE?-EXPRESSION name the rules
   used to parse skippable input in the NAME/s and NAME/?s
   variants. Default to `skippable' and `skippable?' respectively.

   S? and ?S? control which of the NAME/S and NAME/?S rules should be
   generated. Default is generating both.

   DEFINER is the name of the macro used to define the \"main\"
   rule. Defaults to `esrap:defrule'."
  (let+ (((name
           &key
           (s?                    t)
           (skippable-expression  (when s?
                                    (skippable-rule-for-name 'skippable  name)))
           (?s?                   t)
           (skippable?-expression (when ?s?
                                    (skippable-rule-for-name 'skippable? name)))
           (definer               'defrule))
          (ensure-list name-and-options))
         (name/s  (format-symbol *package* "~A/S" name))
         (name/?s (format-symbol *package* "~A/?S" name)))
    `(progn
       (,definer ,name
           ,expression
         ,@options)

       ,@(when s?
           `((defrule ,name/s
                 (and ,name ,skippable-expression)
               (:function first))))

       ,@(when ?s?
           `((defrule ,name/?s
                 (and ,name ,skippable?-expression)
               (:function first)))))))
