(define (stream feasible)
  ;(:state-stream test-feasible
  (:stream test-feasible
    :inputs (?o)
    :domain (Block ?o)
    :fluents (OnTable)
    ;:certified (Feasible1 ?o)

    :outputs (?t)
    :certified (Feasible2 ?o ?t)
  )
)