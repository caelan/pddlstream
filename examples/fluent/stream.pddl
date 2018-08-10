(define (stream feasible)
  (:state-stream test-feasible
    :inputs (?o)
    :fluents (OnTable)
    :certified (Feasible1 ?o)
    ;:outputs (?t)
    ;:certified (Feasible2 ?o ?t)
  )
)