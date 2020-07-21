(define (stream data-network)
	;(:stream test-less_equal
	;	:inputs (?num1 ?num2)
	;	:domain (and (numbers ?num1) (numbers ?num2))
	;	:certified (LESS-EQUAL ?num1 ?num2)
	;)
	;(:stream test-sum ; Could also make this a function
	;	:inputs (?num1 ?num2 ?num3)
	;	:domain (and (numbers ?num1) (numbers ?num2) (numbers ?num3))
	;	:certified (SUM ?num1 ?num2 ?num3)
	;)
	(:stream test-online
		:inputs (?from ?to)
		:domain (CONNECTED ?from ?to)
		:certified (ONLINE ?from ?to)
	)
)
