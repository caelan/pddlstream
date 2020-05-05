(define (stream fault-tolerant)
	(:stream test-open ; Can also negate
		:inputs (?location1 ?location2)
		:domain (Road ?location1 ?location2)
		:certified (Open ?location1 ?location2)
	)
	;(:stream test-package
	;	:inputs (?package ?warehouse)
	;	:domain (and (Package ?package) (Warehouse ?warehouse))
	;	:certified (AtLocation ?package ?warehouse)
	;)
	;(:function (Distance ?location1 ?location2)
	;           (Edge ?location1 ?location2))
)
