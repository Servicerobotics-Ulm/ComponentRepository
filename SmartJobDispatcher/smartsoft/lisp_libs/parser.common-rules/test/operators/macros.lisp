;;;; macros-operators.lisp --- Tests for the operator grammar macros.
;;;;
;;;; Copyright (C) 2013-2018 Jan Moringen
;;;;
;;;; Author: Jan Moringen <jmoringe@techfak.uni-bielefeld.de>

(cl:in-package #:parser.common-rules.operators.test)

(def-suite* :parser.common-rules.operators.macros
  :in :parser.common-rules.operators)

(test define-unary-operator-rule.prefix
  "Test the `define-unary-operator-rule' macro for prefix direction."

  (define-unary-operator-rule minus
      #\- (digit-char-p character)
      :fixity                :prefix
      :skippable?-expression (* #\Space))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (minus)
      ;; Some matching inputs.
      ("1"   #\1
             nil)
      ("-1"  '(:unary-operator
               (:operand ((#\1)))
               :operator "-" :bounds (0 . 2))
             nil)
      ("- 1" '(:unary-operator
               (:operand ((#\1)))
               :operator "-" :bounds (0 . 3))
             nil)

      ;; Some non-matching inputs.
      ("1-"  #\1 1)
      ("1 -" #\1 1))))

(test define-unary-operator-rule.no-skippable
  "Test the `define-unary-operator-rule' macro without a skippable
   expression."

  (define-unary-operator-rule minus/no-skippable
    #\- (digit-char-p character)
    :fixity                :prefix
    :skippable?-expression nil)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (minus/no-skippable)
      ;; Some matching inputs.
      ("-1"  '(:unary-operator
               (:operand ((#\1)))
               :operator "-" :bounds (0 . 2))
             nil)
      ;; Some non-matching inputs.
      ("- 1" nil))))

(test define-unary-operator-rule.postfix
  "Test the `define-unary-operator-rule' macro for postfix direction."

  (define-unary-operator-rule plusplus
      "++" (digit-char-p character)
      :fixity                :postfix
      :skippable?-expression (* #\Space))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (plusplus)
      ;; Some matching inputs.
      ("1"    #\1
              nil)
      ("1++"  '(:unary-operator
                (:operand ((#\1)))
                :operator "++" :bounds (0 . 3))
              nil)
      ("1 ++" '(:unary-operator
                (:operand ((#\1)))
                :operator "++" :bounds (0 . 4))
              nil)

      ;; Some non-matching inputs.
      ("++1"  nil)
      ("++ 1" nil))))

(test define-unary-operator-rule.node-kind
  "Test specifying the node kind in `define-unary-operator-rule'."

  (define-unary-operator-rule custom-minus
      #\- (digit-char-p character)
      :fixity                :prefix
      :skippable?-expression (* #\Space)
      :node-kind             :custom-operator)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (custom-minus)
      ("-1" '(:custom-operator
              (:operand ((#\1)))
              :operator "-" :bounds (0 . 2))
            nil))))

(test define-binary-operator-rule.smoke
  "Smoke test for the `define-binary-operator-rule' macro."

  (define-binary-operator-rule plus
      #\+ (digit-char-p character)
      :skippable?-expression (* #\Space))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (plus)
      ;; Some matching inputs.
      ("1"     #\1)
      ("1+2"   '(:binary-operator
                 (:operand ((#\1) (#\2)))
                 :operator "+" :bounds (0 . 3)))
      ("1+2+3" '(:binary-operator
                 (:operand (((:binary-operator
                              (:operand ((#\1) (#\2)))
                              :operator "+" :bounds (0 . 3)))
                            (#\3)))
                 :operator "+" :bounds (0 . 5)))

      ;; Some non-matching inputs.
      ("1+"    #\1 1 t)
      ("+1"    nil)
      ("1++2"  #\1 1 t))))

(test define-binary-operator-rule.smoke
  "Test the `define-binary-operator-rule' macro without a skippable
   expression."

  (define-binary-operator-rule plus/no-skippable
    #\+ (digit-char-p character)
    :skippable?-expression nil)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (plus/no-skippable)
      ;; Some matching inputs.
      ("1"     #\1)
      ("1+2"   '(:binary-operator
                 (:operand ((#\1) (#\2)))
                 :operator "+" :bounds (0 . 3)))
      ;; Some non-matching inputs.
      ("1 +2"  #\1 1)
      ("1+ 2"  #\1 1)
      ("1 + 2" #\1 1))))

(test define-binary-operator-rule.node-kind
  "Test specifying the node kind in `define-binary-operator-rule'."

  (define-binary-operator-rule custom-plus
      #\+ (digit-char-p character)
      :skippable?-expression (* #\Space)
      :node-kind             :custom-operator)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (custom-plus)
      ("1+2" '(:custom-operator
               (:operand ((#\1) (#\2)))
               :operator "+" :bounds (0 . 3))
             nil))))

(test define-ternary-operator-rule.smoke
  "Smoke test for the `define-ternary-operator-rule' macro."

  (define-ternary-operator-rule if-then-else
      #\? #\: (digit-char-p character)
      :skippable?-expression (* #\Space))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (if-then-else)
      ;; Some matching inputs.
      ("1?2:3"     '(:ternary-operator
                     (:operand ((#\1) (#\2) (#\3)))
                     :operator1 "?" :operator2 ":" :bounds (0 . 5)))
      ("1 ? 2 : 3" '(:ternary-operator
                     (:operand ((#\1) (#\2) (#\3)))
                     :operator1 "?" :operator2 ":" :bounds (0 . 9)))
      ("1?2?3:4:5" '(:ternary-operator
                     (:operand ((#\1)
                                ((:ternary-operator
                                  (:operand ((#\2) (#\3) (#\4)))
                                  :operator1 "?" :operator2 ":" :bounds (2 . 7)))
                                (#\5)))
                     :operator1 "?" :operator2 ":" :bounds (0 . 9)))

      ;; Some non-matching inputs.
      ("1?2"       #\1 1)
      ("1:2"       #\1 1)
      ("1?2:"      #\1 1)
      ("1?2?3"     #\1 1)
      ("1??2:3"    #\1 1)
      (" 1?2:3"    nil))))

(test define-ternary-operator-rule.no-skippable
  "Test the `define-ternary-operator-rule' macro without a skippable
   expression."

  (define-ternary-operator-rule if-then-else/no-skippable
    #\? #\: (digit-char-p character)
    :skippable?-expression nil)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (if-then-else/no-skippable)
      ;; Some matching inputs.
      ("1?2:3"  '(:ternary-operator
                  (:operand ((#\1) (#\2) (#\3)))
                  :operator1 "?" :operator2 ":" :bounds (0 . 5)))
      ;; Some non-matching inputs.
      ("1 ?2:3" #\1 1)
      ("1? 2:3" #\1 1)
      ("1?2 :3" #\1 1)
      ("1?2: 3" #\1 1))))

(test define-ternary-operator-rule.node-kind
  "Test specifying the node kind in `define-ternary-operator-rule'."

  (define-ternary-operator-rule custom-if-then-else
      #\? #\: (digit-char-p character)
      :skippable?-expression (* #\Space)
      :node-kind             :custom-operator)

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (custom-if-then-else)
      ("1?2:3" '(:custom-operator
                 (:operand ((#\1) (#\2) (#\3)))
                 :operator1 "?" :operator2 ":" :bounds (0 . 5))
               nil))))

(test define-operator-rules.syntax-errors
  "Test errors signaled for syntactically invalid
   `define-operator-rules' forms."

  (flet ((test-case (form)
           (signals error (eval form))))
    ;; Missing operators and leaf expression.
    (test-case '(define-operator-rules (:skippable?-expression #\Space)))
    ;; Missing operators.
    (test-case '(define-operator-rules (:skippable?-expression #\Space)
                  leaf))

    ;; Invalid arity.
    (test-case '(define-operator-rules (:skippable?-expression #\Space)
                  (0 foo "+")
                  leaf))
    ;; Invalid associativity.
    (test-case '(define-operator-rules (:skippable?-expression #\Space)
                  (2 foo "+" :associativity :foo)
                  leaf))))

(test define-operator-rules.smoke
  "Smoke test for the `define-operator-rules' macro."

  (define-operator-rules (:skippable?-expression (* #\Space))
    (2 assign       ":="    :associativity :none)
    (3 if-then-else "?" ":")
    (2 term         "+")
    (2 factor       "*")
    (2 expon        "^"     :associativity :right)
    (1 neg          "-")
    (1 inc          "++"    :fixity :postfix)
    (digit-char-p character))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (assign)
      ;; Some matching inputs.
      ("1"          #\1)
      ("1:=2"       '(:binary-operator
                      (:operand ((#\1) (#\2)))
                      :operator ":=" :bounds (0 . 4)))
      ("1:=2+3^4^5" '(:binary-operator
                      (:operand ((#\1)
                                 ((:binary-operator
                                   (:operand ((#\2)
                                              ((:binary-operator
                                                (:operand ((#\3)
                                                           ((:binary-operator
                                                             (:operand ((#\4) (#\5)))
                                                             :operator "^" :bounds (7 . 10)))))
                                                :operator "^" :bounds (5 . 10)))))
                                   :operator "+" :bounds (3 . 10)))))
                      :operator ":=" :bounds (0 . 10)))
      ("2+5*1++"   '(:binary-operator
                     (:operand ((#\2)
                                ((:binary-operator
                                  (:operand ((#\5)
                                             ((:unary-operator
                                               (:operand ((#\1)))
                                               :operator "++" :bounds (4 . 7)))))
                                  :operator "*" :bounds (2 . 7)))))
                     :operator "+" :bounds (0 . 7)))
      ("1?2:3"     '(:ternary-operator
                     (:operand ((#\1) (#\2) (#\3)))
                     :operator1 "?" :operator2 ":" :bounds (0 . 5)))

      ;; Some non-matching inputs.
      ("1:=2:=3"   '(:binary-operator
                     (:operand ((#\1) (#\2)))
                     :operator ":=" :bounds (0 . 4))
                   4 t))))

(test define-operator-rules.parentheses
  "Smoke test added support for parenthesis to the output of the
   `define-operator-rules' macro."

  (define-operator-rules (:skippable?-expression (* #\Space))
    (2 assign ":=" :associativity :none)
    (2 term   "+")
    (2 factor "*")
    (2 expon  "^" :associativity :right)
    (1 neg    "-")
    (1 inc    "++" :fixity :postfix)
    (or parentheses (digit-char-p character)))

  (esrap:defrule parentheses
      (and #\( assign #\))
    (:function second))

  (architecture.builder-protocol:with-builder ('list)
    (parses-are (assign)
      ;; Some matching inputs.
      ("1"           #\1)
      ("(1)"         #\1)
      ("((1))"       #\1)
      ("(1:=2)*3"    '(:binary-operator
                       (:operand (((:binary-operator
                                    (:operand ((#\1) (#\2)))
                                    :operator ":=" :bounds (1 . 5)))
                                  (#\3)))
                       :operator "*" :bounds (0 . 8)))
      ("1*(2+5)"     '(:binary-operator
                       (:operand ((#\1)
                                  ((:binary-operator
                                    (:operand ((#\2) (#\5)))
                                    :operator "+" :bounds (3 . 6)))))
                       :operator "*" :bounds (0 . 7)))
      ("((2+5)^1)^2" '(:binary-operator
                       (:operand (((:binary-operator
                                    (:operand (((:binary-operator
                                                 (:operand ((#\2) (#\5)))
                                                 :operator "+" :bounds (2 . 5)))
                                               (#\1)))
                                    :operator "^" :bounds (1 . 8)))
                                  (#\2)))
                       :operator "^" :bounds (0 . 11)))
      ("(1)++++"      '(:unary-operator
                        (:operand (((:unary-operator
                                     (:operand ((#\1)))
                                     :operator "++" :bounds (0 . 5)))))
                        :operator "++" :bounds (0 . 7)))

      ;; Some non-matching inputs.
      ("(1"          nil)
      ("1)"          #\1 1 t)
      ("1(+2)"       #\1 1 t))))
