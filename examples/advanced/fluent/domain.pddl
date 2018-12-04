(define (domain fluent)
  (:requirements :strips)
  (:predicates
    (Block ?o)
    (OnTable ?o)
    (Holding ?o)
    (Feasible1 ?o)
    (Feasible2 ?o ?t)
    (Cleanable ?o)
    (Clean ?o)
    (Cooked ?o)
  )
  ;(:action pick1
  ;  :parameters (?o)
  ;  :precondition (and (OnTable ?o) (Feasible1 ?o))
  ;  :effect (and (Holding ?o) (not (OnTable ?o))))
  (:action pick2
    :parameters (?o ?t)
    :precondition (and (OnTable ?o) (Feasible2 ?o ?t))
    :effect (and (Holding ?o) (not (OnTable ?o))))

  (:action clean
    :parameters (?o)
    :precondition (Cleanable ?o)
    :effect (Clean ?o))
  (:action cook
    :parameters (?o)
    :precondition (and (Block ?o) (Clean ?o))
    :effect (and (Cooked ?o) (not (Clean ?o))))
)