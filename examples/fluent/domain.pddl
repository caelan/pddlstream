(define (domain fluent)
  (:requirements :strips)
  (:predicates
    (OnTable ?o)
    (Holding ?o)
    (Feasible1 ?o)
    (Feasible2 ?o ?t)
    (Test)
  )
  (:action pick1
    :parameters (?o)
    :precondition (and (OnTable ?o) (Feasible1 ?o))
    :effect (and (Holding ?o) (not (OnTable ?o))))
  (:action pick2
    :parameters (?o ?t)
    :precondition (and (OnTable ?o) (Feasible2 ?o ?t))
    :effect (and (Holding ?o) (not (OnTable ?o))))
  (:action default
    :effect (Test)
  )
)