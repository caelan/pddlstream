(define (stream no-mystery-prime-strips)
	(:stream test-open
		:inputs (?n1 ?n2)
		:domain (connected ?n1 ?n2)
		:certified (open ?n1 ?n2)
	)
)
