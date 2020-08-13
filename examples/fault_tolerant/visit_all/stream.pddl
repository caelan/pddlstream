(define (stream grid-visit-all)
	(:stream test-open
		:inputs (?curpos ?nextpos)
		:domain (connected ?curpos ?nextpos)
		:certified (open ?curpos ?nextpos)
	)
)
