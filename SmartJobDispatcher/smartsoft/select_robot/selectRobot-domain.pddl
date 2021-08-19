(define (domain select-robot)
 (:requirements :adl :typing)
  (:types robot task)
 
  (:predicates
     (robot ?robot - robot)
     (task ?task - task)
     (is-docked ?robot - robot)
     (performs-task ?robot - robot)
     (is-performed ?task - task))
  (:functions 
     (distance ?robot ?task)
     (undocking-cost)
     (total-cost))
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; actions

  (:action undock-robot
    :parameters (?robot - robot)
    :precondition (and 
                      (is-docked ?robot))
    :effect (and 
                (not (is-docked ?robot))
                (increase (total-cost) (undocking-cost))))

  (:action perform-task
    :parameters (?robot - robot ?task - task)
    :precondition (and
                     (not (is-docked ?robot))
                     (not (performs-task ?robot))
                     (not (is-performed ?task)))
    :effect (and (performs-task ?robot)
                 (is-performed ?task)
                 (increase (total-cost) (distance ?robot ?task)))))
