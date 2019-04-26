;;--------------------------------------------------------------------------
;;
;;  Copyright (C) 	2011 Andreas Steck
;;                      2016 Matthias Lutz
;;
;;
;;      ZAFH Servicerobotic Ulm
;;      Christian Schlegel
;;	University of Applied Sciences
;;      Prittwitzstr. 10
;;      89075 Ulm
;;      Germany
;;
;;
;;  This library is free software; you can redistribute it and/or
;;  modify it under the terms of the GNU Lesser General Public
;;  License as published by the Free Software Foundation; either
;;  version 2.1 of the License, or (at your option) any later version.
;;
;;  This library is distributed in the hope that it will be useful,
;;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;;  Lesser General Public License for more details.
;;
;;  You should have received a copy of the GNU Lesser General Public
;;  License along with this library; if not, write to the Free Software
;;  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
;;
;;--------------------------------------------------------------------------

;;--------------------------------------------------------------------------
;;
;; CREDITS:
;;
;; The code for unification is taken from the SimpleAgenda written by 
;; Christian Schlegel (schlegel@hs-ulm.de).
;;
;;--------------------------------------------------------------------------



;;; Identification of symbols

;;; VARIABLEP returns T if X is a symbol whose name begins with "?"
(defun variablep (x)
  (and (symbolp x) (equal (elt (symbol-name x) 0) #\?)))

;;; EXTRACT-VARIABLES returns a list of all of the variables in EXPR
(defun extract-variables (expr)
  (cond ((variablep expr) (list expr))
        ((consp expr) (my-union (extract-variables (car expr))
                             (extract-variables (cdr expr))))))

;; GET-VARIABLE-NAME removes the ? from symbol eg.: ?var1 --> var1
(defun get-variable-name (x)
   (read-from-string (string-left-trim "?" (symbol-name x))))

;;; Basic set operations

;;; Lisp has a built-in UNION function, but we need something that returns
;;; the same results regardless of what platform this code is running on
(defun my-union (set1 set2 &key (test #'eql))
  (dolist (x set1)
    (pushnew x set2 :test test))
  set2)

;;; Predicate to tell whether two sets contain the same elements
(defun same-set-p (x y)
  (and (subsetp x y :test #'equal)
       (subsetp y x :test #'equal)))




;;; Substitutions and unification

;;; APPLY-SUBSTITUTION searches through TARGET, replacing each variable
;;; symbol with the corresponding value (if any) in A-LIST
(defun apply-substitution (target a-list)
  (cond ((variablep target)
         (let ((result (assoc target a-list)))
           (if result (cdr result) target)))
        ((atom target) target)
        (t (cons (apply-substitution (car target) a-list)
                 (apply-substitution (cdr target) a-list)))))

;;; COMPOSE-SUBSTITUTIONS applies SUB2 to the right-hand-side of each item
;;; in SUB1, and appends all items in SUB2 whose left-hand-sides aren't in SUB1.
;;; *** Warning:  COMPOSE-SUBSTITUTIONS destroys the old value of SUB1 ***
;;; I normally would avoid destructive operations, but here it speeds up the
;;; planner by about 5%, and none of the calling functions need SUB1 afterwards
(defun compose-substitutions (sub1 sub2)
  (dolist (pair sub1)
    (setf (cdr pair) (apply-substitution (cdr pair) sub2)))
  (dolist (pair sub2 sub1)
    (pushnew pair sub1 :test #'(lambda (x y) (equal (car x) (car y))))))

;;; UNIFY is based on the procedure in Nilsson's 1980 book, but modified
;;; to produce an mgu that simultaneously unifies and standardizes the two
;;; expressions.  This improves efficiency by avoiding repeated calls to
;;; STANDARDIZER in FIND-SATISFIERS and APPLY-METHOD.
(defun unify (e1 e2)
  (cond ((atom e1) (unify1 e1 e2))
        ((atom e2) (unify1 e2 e1))
        (t (let ((hsub (unify (car e1) (car e2))))
             (if (equal hsub 'fail)
               'fail
               (let* ((tail1 (apply-substitution (cdr e1) hsub))
                      (tail2 (apply-substitution (cdr e2) hsub))
                      (tsub (unify tail1 tail2)))
                 (if (equal tsub 'fail)
                   'fail
                   (compose-substitutions hsub tsub))))))))

;;; UNIFY1 is the "base case", called when e1 is an atom.
(defun unify1 (e1 e2)
  (cond ((equal e1 e2) (standardizer e1))
        ((variablep e1)
         (if (occurs e1 e2)
           'fail
           (compose-substitutions (list (cons e1 e2)) (standardizer e2))))
        ((variablep e2)
         (compose-substitutions (list (cons e2 e1)) (standardizer e1)))
        (t 'fail)))

;;; OCCURS is the infamous "occurs check" - it returns T if
;;; the variable symbol V occurs anywhere in the expression EXPR
(defun occurs (v expr)
  (if (atom expr)
    (eq v expr)
    (or (occurs v (car expr)) (occurs v (cdr expr)))))

;;; STANDARDIZER returns a substitution that replaces every variable symbol
;;; in EXPRESSION with a new variable symbol not used elsewhere
(defun standardizer (expression)
  (mapcar #'(lambda (x) (cons x (gensym "?"))) (extract-variables expression)))

;;; STANDARDIZE returns the result of applying STANDARDIZER to EXPR
(defun standardize (expr)
  (apply-substitution expr (standardizer expr)))


(defun split-namespace (instring)
    "Returns a list of substrings of string
divided by ONE . .
Note: Two consecutive . will be seen as
if there were an empty string between them."
  (let ((dot nil))
  (setf dot (position #\. instring :test #'equalp))
  (if (null dot)
    (list nil (intern instring))
    (list (intern (subseq instring 0 dot)) (intern (subseq instring (+ dot 1))) ))))

(defun parse-tcl-signature (index)
    (let* ((split-res (split-namespace (string (first index))))
           (tcb-name (second split-res))
           (namespace-name (first split-res)))
      ;;FALSE DO NOT TOUCH THE VALUE OF INDEX!
      ;;;(setf (car index) tcb-name)
      ;;
      (let ((goal-name tcb-name))
    (do ((vars-left (cdr index) (cdr vars-left))
          (in-vars '()) )
      ((null vars-left) (values goal-name namespace-name (nreverse in-vars) '()))
      (if  (eq (car vars-left) '=>)
        (return (values goal-name namespace-name  (nreverse in-vars) (cdr vars-left)))
        (push (car vars-left) in-vars))))))


;;TODO Which is the most up to date version????
;; is this version still used -- if not remove!
;(defun append-prefix-to-each-entry (full-list prefix)
; ;(format t "Input: ~a~%" full-list)
; (loop
;   for item in full-list
;   ;do (format t "Item: ~a~%" item)
;   when (not(null item))
;   collect (append (list (intern (concatenate 'string (symbol-name prefix) "." (symbol-name (first item)) ))) (rest item))))

(defun append-prefix-to-each-entry (full-list prefix)
 ;(format t "Input: ~a~%" full-list)
 (loop
   for item in full-list
   ;do (format t "Item: ~a~%" item)
   when (not(null item))
   collect (append-prefix item prefix)))


(defun append-prefix (entry prefix)
 ;(format t "Entry: ~a~%" entry)
  (if (null entry)
   nil
   (append (list (intern (concatenate 'string (symbol-name prefix) "." (symbol-name (first entry)) ))) (rest entry))))

; (append-prefix '() 'INST2)
; (append-prefix '(abc) 'INST2)
; (append-prefix-to-each-entry '((abc x y)(def)) 'INST2)
; (append-prefix-to-each-entry '((abc x y => x "Matthias" 2)(def)()) 'INST2)
; (append-prefix-to-each-entry '((abc)) 'INST2)



;;; THE FOLLOWING IS USED FOR THE SKILL INTERFACE 


(defun equal-symbol-string (a b)
"This helper function comares two elements string case insenstive, both could be symbols or strings"
  (equalp (if (symbolp a) (symbol-name a) a)  (if (symbolp b) (symbol-name b) b)))

;(defun concatenate-objects-to-symbol (&rest objects)
;  (intern (apply #'concatenate 'string (mapcar #'princ-to-string objects))))

;(parse-tcl-signature '(TCB-TEST2))
;(parse-tcl-signature '(TCB-TEST2 2 ))
;(parse-tcl-signature '(TCB-TEST2 2 2 3 ))
;(parse-tcl-signature '(TCB-TEST2 :ID 2))
;(parse-tcl-signature '(TCB-TEST2 1 :ID 2))
;(parse-tcl-signature '(TCB-TEST2 1 2 3 :ID 2))
;(parse-tcl-signature '(TCB-TEST2 => ?out))
;(parse-tcl-signature '(TCB-TEST2 => ?out ?out2 ?out3))
;(parse-tcl-signature '(TCB-TEST2 => ?out :ID 2))
;(parse-tcl-signature '(TCB-TEST2 => ?out ?out2 ?out3 :ID 2))
;(parse-tcl-signature '(TCB-TEST2 1 2 3 => ?out ?out1 :ID 2))
 ; (parse-tcl-signature '(tcb-first-level 3 :ID 2))

;(defun parse-tcl-signature (index)
;  (let ((goal-name (car index)))
;    (do ((vars-left (cdr index) (cdr vars-left))
;         (in-vars '())
;         (out-vars '())
;         (out-found nil))
;      ((null vars-left) (values goal-name (nreverse in-vars) (nreverse out-vars) '()))
;      (cond 
;        ((eq (car vars-left) '=>)
;          (setf out-found T))
;        ((eq (car vars-left) ':ID)
;          (return (values goal-name (nreverse in-vars) (nreverse out-vars) (cdr vars-left))))
;        (out-found
;          (push (car vars-left) out-vars))
;        (T
;          (push (car vars-left) in-vars))))))

