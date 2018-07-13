(define (domain fluent)
  (:requirements :strips)
  (:predicates
    (OnTable ?x)
    (Holding ?x)
    (Feasible ?x)
  )
  (:action pick
    :parameters (?o)
    :precondition (and (OnTable ?o) (Feasible ?o))
    :effect (and (Holding ?o)
                 (not (OnTable ?o))))
)