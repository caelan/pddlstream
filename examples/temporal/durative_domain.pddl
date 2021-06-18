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

  (:durative-action cook
   :parameters (?f ?s)
   :duration (= ?duration (CookDuration ?f ?s))
   :condition (and
     (at start (Food ?f))
     (at start (Stove ?s))
     (at start (not (Locked ?f)))
     (at start (not (Locked ?s)))
   )
   :effect (and
     (at start (Locked ?f))
     (at start (Locked ?s))
     (at end (not (Locked ?f)))
     (at end (not (Locked ?s)))
     (at end (Cooked ?f))
   )
  )
)