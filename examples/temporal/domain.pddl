(define (domain temporal)
  (:predicates
    (Time ?t)
    (Duration ?dt)
    (Sum ?t1 ?dt ?t2)
    (GE ?t1 ?t2)

    (Food ?f)
    (Stove ?s) ; TODO: Oven

    (CookDuration ?dt ?f ?s)
    (Cooking ?t ?f ?s)

    (AtTime ?t)
    (Cooked ?f)
    (Locked ?s)
    (Premature ?t)
    (Invalid)
    (CanWait)
  )
  (:functions
    (Elapsed ?dt)
    (Difference ?t2 ?t1)
  )

  ;(:action wait
  ;  :parameters (?t1 ?dt ?t2)
  ;  :precondition (and (Sum ?t1 ?dt ?t2)
  ;                     (AtTime ?t1)
  ;                     (CanWait)
  ;                     (not (Premature ?t2))
  ;                     (not (Invalid))
  ;                )
  ;  :effect (and (AtTime ?t2)
  ;               (not (AtTime ?t1))
  ;               (not (CanWait))
  ;               (increase (total-cost) (Elapsed ?dt)))
  ;)

  (:action wait ; # TODO: allow jumping between any times or just between adjacent ones
    :parameters (?t1 ?t2)
    :precondition (and (GE ?t2 ?t1) (not (= ?t1 ?t2))
                       (AtTime ?t1)
                       (CanWait)
                       (not (Premature ?t2))
                       (not (Invalid))
                  )
    :effect (and (AtTime ?t2)
                 (not (AtTime ?t1))
                 (not (CanWait))
                 ;(increase (total-cost) 1)
                 (increase (total-cost) (Difference ?t2 ?t1))
            ))

  (:action start-cooking
    :parameters (?t1 ?dt ?t2 ?f ?s)
    :precondition (and ;(Food ?f) (Stove ?s)
                       (CookDuration ?dt ?f ?s)
                       (Sum ?t1 ?dt ?t2)
                       (AtTime ?t1)
                       (not (Locked ?f)) (not (Locked ?s))
                       ;(not (Premature ?t1))
                       (not (Invalid))
                   )
    :effect (and (Cooking ?t2 ?f ?s)
                 (Locked ?f) (Locked ?s)
                 (CanWait)
                 (increase (total-cost) 0))
  )
  ; TODO: while cooking
  (:action stop-cooking
    :parameters (?t2 ?f ?s)
    :precondition (and (Time ?t2) (Food ?f) (Stove ?s)
                       (Cooking ?t2 ?f ?s)
                       (AtTime ?t2)
                       ;(not (Premature ?t2))
                       (not (Invalid))
                  )
    :effect (and (Cooked ?f)
                 (not (Cooking ?t2 ?f ?s))
                 (not (Locked ?f)) (not (Locked ?s))
                 (CanWait)
                 (increase (total-cost) 0))
  )

  ;# TODO: overall conditions
  ;(:derived (Valid) (and
  ;  (forall (?t ?f ?s) (imply (Cooking ?t ?f ?s)
  ;                            (and ...)))
  ;))

  ;(:derived (Invalid) (or
  ;  (exists (?t ?f ?s) (and (Cooking ?t ?f ?s)
  ;                           ...))
  ;))

  (:derived (Premature ?t2)
    (exists (?t1) (and (GE ?t2 ?t1) (not (= ?t2 ?t1)) (or ; TODO: strictly greater than?
      (exists (?f ?s) (and (Food ?f) (Stove ?s)
                           (Cooking ?t1 ?f ?s)))
  ))))
)