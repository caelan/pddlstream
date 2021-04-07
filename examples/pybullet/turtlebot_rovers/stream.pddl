(define (stream rovers)

  ;(:stream test-reachable ; Eager
  ;  :inputs (?v ?q)
  ;  :domain (Conf ?v ?q)
  ;  :certified (Reachable ?v ?q)
  ;)
  (:stream test-cfree-ray-conf
    :inputs (?y ?v ?q)
    :domain (and (Ray ?y) (Conf ?v ?q))
    :certified (CFreeRayConf ?y ?v ?q)
  )
  ;(:stream obj-range ; Eager
  ;  :inputs (?q ?o)
  ;  :domain (and (Conf ?q) (Objective ?o))
  ;  :certified (ImageRange ?q ?o)
  ;)
  ;(:stream com-range ; Eager
  ;  :inputs (?q ?l)
  ;  :domain (and (Conf ?q) (Lander ?l))
  ;  :certified (ComRange ?q ?l)
  ;)
  (:stream obj-inv-visible
    :inputs (?v ?o)
    :domain (and (Rover ?v) (Objective ?o))
    :outputs (?q ?y)
    :certified (and (ImageVisible ?v ?q ?y ?o) (ImageRange ?v ?q ?o)
                    (Conf ?v ?q) (Ray ?y))
  )
  (:stream com-inv-visible
    :inputs (?v ?l)
    :domain (and (Rover ?v) (Lander ?l))
    :outputs (?q ?y)
    :certified (and (ComVisible ?v ?q ?y ?l) (ComRange ?v ?q ?l)
                    (Conf ?v ?q) (Ray ?y))
  )
  (:stream sample-above
    :inputs (?v ?r)
    :domain (and (Rover ?v) (Rock ?r))
    :outputs (?q)
    :certified (and (Above ?v ?q ?r) (Conf ?v ?q))
  )
  (:stream sample-motion
    :inputs (?v ?q1 ?q2)
    ; :domain (and (Reachable ?v ?q1) (Reachable ?v ?q2))
    :domain (and (Conf ?v ?q1) (Conf ?v ?q2))
    :outputs (?t)
    :certified (Motion ?v ?q1 ?t ?q2)
  )

  ; TODO: could make the robots communicate via line of sight to each other
)
