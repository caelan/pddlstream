(define (stream rovers)

  ;(:stream test-reachable ; Eager
  ;  :inputs (?r ?q)
  ;  :domain (Conf ?r ?q)
  ;  :certified (Reachable ?r ?q)
  ;)
  (:stream test-cfree-ray-conf
    :inputs (?y ?r ?q)
    :domain (and (Ray ?y) (Conf ?r ?q))
    :certified (CFreeRayConf ?y ?r ?q)
  )
  ;(:stream image-range ; Eager
  ;  :inputs (?q ?o)
  ;  :domain (and (Conf ?q) (Objective ?o))
  ;  :certified (ImageRange ?q ?o)
  ;)
  ;(:stream transmit-range ; Eager
  ;  :inputs (?q ?l)
  ;  :domain (and (Conf ?q) (Lander ?l))
  ;  :certified (ComRange ?q ?l)
  ;)
  (:stream inv-image
    :inputs (?r ?o)
    :domain (and (Rover ?r) (Objective ?o))
    :outputs (?q ?y)
    :certified (and (ImageConf ?r ?q ?y ?o) (ImageRange ?r ?q ?o)
                    (Conf ?r ?q) (Ray ?y))
  )
  (:stream inv-transmit
    :inputs (?r ?l)
    :domain (and (Rover ?r) (Lander ?l))
    :outputs (?q ?y)
    :certified (and (TransmitConf ?r ?q ?y ?l) (ComRange ?r ?q ?l)
                    (Conf ?r ?q) (Ray ?y))
  )
  (:stream sample-above
    :inputs (?r ?g)
    :domain (and (Rover ?r) (Rock ?g))
    :outputs (?q)
    :certified (and (Above ?r ?q ?g) (Conf ?r ?q))
  )
  (:stream sample-motion
    :inputs (?r ?q1 ?q2)
    ; :domain (and (Reachable ?r ?q1) (Reachable ?r ?q2))
    :domain (and (Conf ?r ?q1) (Conf ?r ?q2))
    :outputs (?t)
    :certified (Motion ?r ?q1 ?t ?q2)
  )
  ; TODO: could make the robots communicate via line of sight to each other
)
