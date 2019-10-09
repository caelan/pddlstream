; https://github.mit.edu/caelan/stripstream/blob/master/stripstream/fts/examples/unfactored_tamp.py
(define (domain rmmp)
  (:requirements :strips :equality)
  (:predicates
    ; Static predicates
    (CanMove)
    (Conf ?m ?q)
    (Mode ?m)
    (Adjacent ?m1 ?m2)
    (Motion ?m ?q1 ?t ?q2)
    (Switch ?m1 ?m2 ?q)
    (GoalState ?m ?q)

    ; Fluent predicates
    (AtMode ?m)
    (AtConf ?q)
  )

  (:action move
    :parameters (?m ?q1 ?q2 ?t)
    :precondition (and (Motion ?m ?q1 ?t ?q2)
                       (AtMode ?m) (AtConf ?q1)
                       (CanMove)
                  )
    :effect (and (AtConf ?q2)
                 (not (CanMove))
                 (not (AtConf ?q1))))

  (:action switch
    :parameters (?m1 ?m2 ?q)
    :precondition (and (Switch ?m1 ?m2 ?q)
                       (AtMode ?m1) (AtConf ?q))
    :effect (and (AtMode ?m2) (CanMove)
                 (not (AtMode ?m1))))
)