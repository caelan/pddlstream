(define (stream feasible)
  (:stream test-feasible
    :inputs (?o)
    :domain (Block ?o)
    :fluents (OnTable)
    ;:certified (Feasible1 ?o)
    :outputs (?t)
    :certified (Feasible2 ?o ?t)
  )

  (:stream test-cleanable
    :inputs (?o)
    :domain (Block ?o)
    :fluents (Cooked)
    :certified (Cleanable ?o)
  )
)