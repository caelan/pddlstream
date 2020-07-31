(define (stream rohanblocks)

;; Streams for sampling types

(:stream sample-robot
  :outputs (?robot)
  :certified (isrobot ?robot)
)

(:stream sample-block
  :outputs (?x)
  :certified (isblock ?x)
)

(:stream sample-goal-block
  :outputs (?x)
  :certified (isblock ?x)
)

;; Streams for testing predicates

(:stream test-on
  :inputs (?x ?y)
  :domain (and (isblock ?x) (isblock ?y))
  :certified (on ?x ?y)
)

(:stream test-ontable
  :inputs (?x)
  :domain (isblock ?x)
  :certified (ontable ?x)
)

(:stream test-clear
  :inputs (?x)
  :domain (isblock ?x)
  :certified (clear ?x)
)

(:stream test-handempty
  :inputs (?robot)
  :domain (isrobot ?robot)
  :certified (handempty ?robot)
)

(:stream test-handfull
  :inputs (?robot)
  :domain (isrobot ?robot)
  :certified (handfull ?robot)
)

(:stream test-holding
  :inputs (?x)
  :domain (isblock ?x)
  :certified (holding ?x)
)

;; Streams for testing action predicates

(:stream test-pickup
  :inputs (?x)
  :domain (isblock ?x)
  :certified (pickup ?x)
)

(:stream test-putdown
  :inputs (?x)
  :domain (isblock ?x)
  :certified (putdown ?x)
)

(:stream test-stack
  :inputs (?x ?y)
  :domain (and (isblock ?x) (isblock ?y))
  :certified (stack ?x ?y)
)

(:stream test-unstack
  :inputs (?x)
  :domain (isblock ?x)
  :certified (unstack ?x)
)

)