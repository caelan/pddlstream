(define (stream grid-visit-all)
	(:stream test-open
		:inputs (?x ?y ?z)
		:domain (can_traverse ?x ?y ?z)
		:certified (open ?x ?y ?z)
	)
)
