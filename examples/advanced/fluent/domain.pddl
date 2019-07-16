(define (domain fluent)
  (:requirements :strips)
  (:predicates
    (Block ?b)
    (OnTable ?b)
    (Holding ?b)
    (Pickable ?b ?t)
    (Cleanable ?b ?t)
    (Clean ?b)
    (Cooked ?b)
  )
  (:action pick
    :parameters (?b ?t)
    :precondition (and (OnTable ?b) (Pickable ?b ?t))
    :effect (and (Holding ?b) (not (OnTable ?b))))

  (:action clean
    :parameters (?b ?t)
    :precondition (Cleanable ?b ?t)
    :effect (Clean ?b))
  (:action cook
    :parameters (?b)
    :precondition (and (Block ?b) (Clean ?b))
    :effect (and (Cooked ?b) (not (Clean ?b))))
)