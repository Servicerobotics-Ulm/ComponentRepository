(define (problem select-robot-1)
  (:domain select-robot)
    (:objects
      ROBOT-1 - robot
      ROBOT-2 - robot
      ROBOT-3 - robot
      TASK-1 - task
      TASK-2 - task
    )

    (:init
      (= (total-cost) 0)
      (= (undocking-cost) 10)
      (= (distance ROBOT-1 TASK-1) 1)
      (= (distance ROBOT-1 TASK-2) 2)
      (= (distance ROBOT-2 TASK-1) 2)
      (= (distance ROBOT-2 TASK-2) 20)
      (= (distance ROBOT-3 TASK-1) 1)
      (= (distance ROBOT-3 TASK-2) 2)
      (is-docked ROBOT-1)
      (is-docked ROBOT-3)
      (robot ROBOT-1)
      (robot ROBOT-2)
      (robot ROBOT-3)
      (task TASK-1)
      (task TASK-2))

    (:goal
        (and (is-performed TASK-1)
             (is-performed TASK-2)))

    (:metric minimize (total-cost)))
