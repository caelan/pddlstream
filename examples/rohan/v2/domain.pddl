(define (domain tom)
  (:requirements :strips)
  (:predicates
    ; Static predicates
    (Action ?a)
    (IsValidPhysics ?s ?a ?ns)
    (IsGoalState ?s)
    (State ?s)

    ; Fluent predicates
    (CurrentState ?s)
    (AtGoal)
  )

  (:action interact
    :parameters (?s ?a ?ns)
    :precondition (and (CurrentState ?s)
                       (IsValidPhysics ?s ?a ?ns))
    :effect (and (not (CurrentState ?s))
                 (CurrentState ?ns)))

  (:action finish
    :parameters (?s)
    :precondition (and (CurrentState ?s)
                       (IsGoalState ?s))
    :effect (and
                 (AtGoal)))

)