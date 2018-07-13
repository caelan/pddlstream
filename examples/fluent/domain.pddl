(define (domain fluent)
  (:requirements :strips)
  (:predicates
    (OnTable ?x)
    (Holding ?x)
    (Feasible)
  )
  (:action pick
    :parameters (?o)
    :precondition (and (OnTable ?o) (Feasible))
    :effect (and (Holding ?o)
                 (not (OnTable ?o))))
)