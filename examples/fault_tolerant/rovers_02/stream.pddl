(define (stream grid-visit-all)
	(:stream test-open
		:inputs (?y ?z)
		:domain (and (waypoint ?y) (waypoint ?z))
		:certified (open ?y ?z)
	)
)
