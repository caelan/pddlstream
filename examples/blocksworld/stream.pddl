(define (stream rohanblocks)

;(:stream sample-block
;  :outputs (?x)
;  :certified (isblock ?x)
;)

;(:stream test-block
;  :inputs (?b)
;  :domain (object ?b)
;  :certified (isblock ?b)
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

(:stream sample-on
  :inputs (?b1)
  :domain (isblock ?b1)
  :outputs (?b2)
  :certified (and (on ?b2 ?b1)
                  (isblock ?b2))
)

(:stream test-clear
  :inputs (?b)
  :domain (isblock ?b)
  :certified (clear ?b)
)

;(:stream test-arm-empty
;  :certified (arm-empty)
;)

)