(define (stream rohanblocks)

;(:stream sample-block
;  :outputs (?x)
;  :certified (isblock ?x)
;)

;(:stream test-on
;  :inputs (?x ?y)
;  :domain (and (isblock ?x) (isblock ?y))
;  :certified (on ?x ?y)
;)

;(:stream test-ontable
;  :inputs (?x)
;  :domain (isblock ?x)
;  :certified (ontable ?x)
;)

(:stream test-clear
  :inputs (?b)
  :domain (isblock ?b)
  :certified (clear ?b)
)

(:stream test-arm-empty
  :certified (arm-empty)
)

)