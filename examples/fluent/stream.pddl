(define (stream feasible)
  (:state-stream test-feasible
    :inputs (?o)
    :fluents (OnTable)
    :certified (Feasible ?o)
  )
)