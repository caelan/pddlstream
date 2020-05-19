(define (stream risk)
	(:stream test-connected
		:inputs (?x ?y)
		:domain (and (OBJECT ?x) (OBJECT ?y))
		:certified (CONNECTED ?x ?y)
	)
)
