(define (stream pick-and-place)
  (:stream ge
    :inputs (?d ?l ?p)
    :domain (and (Dist ?d) (Location ?l) (Prob ?p))
    :outputs ()
    :certified (GE ?d ?l ?p)
  )
  (:stream prob-after-move
    :inputs (?l1 ?l2 ?d1)
    :domain (and (Location ?l1) (Location ?l2) (Dist ?d1))
    :outputs (?d2)
    :certified (and (Dist ?d2) (MoveProb ?l1 ?l2 ?d1 ?d2))
  )
  (:stream prob-after-look
    :inputs (?l ?d1)
    :domain (and (Location ?l) (Dist ?d1))
    :outputs (?d2)
    :certified (and (Dist ?d2) (LookProb ?l ?d1 ?d2))
  )

)