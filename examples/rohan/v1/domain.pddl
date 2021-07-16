(define (domain tom)
  (:requirements :strips)
  (:predicates
    ; Static predicates
    (State ?s)
    (Action ?a)
    (IsValidPhysics ?s ?a ?ns)

    ; Fluent predicates
    (CurrentState ?s)
  )

  (:action interact
    :parameters (?s ?a ?ns)
    :precondition (and (State ?s) (Action ?a) (CurrentState ?s)
                       (IsValidPhysics ?s ?a ?ns))
    :effect (and (not (CurrentState ?s))
                 (CurrentState ?ns)))

)