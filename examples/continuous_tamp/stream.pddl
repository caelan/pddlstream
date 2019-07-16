(define (stream continuous-tamp)
  (:stream s-region
    :inputs (?b ?r)
    ;:domain (and (Block ?b) (Region ?r))
    :domain (Placeable ?b ?r)
    :outputs (?p)
    :certified (and (Pose ?b ?p) (Contain ?b ?p ?r)))
  (:stream s-ik
    :inputs (?b ?p ?g)
    :domain (and (Pose ?b ?p) (Grasp ?b ?g))
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?b ?q ?p ?g)))
  (:stream s-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (Traj ?t) (Motion ?q1 ?t ?q2)))

  (:stream t-cfree
    :inputs (?b1 ?p1 ?b2 ?p2)
    :domain (and (Pose ?b1 ?p1) (Pose ?b2 ?p2))
    :certified (CFree ?b1 ?p1 ?b2 ?p2))
  (:stream t-region
    :inputs (?b ?p ?r)
    :domain (and (Pose ?b ?p) (Placeable ?b ?r))
    :certified (Contain ?b ?p ?r))

  (:function (Dist ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2)) ; TODO: augment this with the keyword domain
  )
  (:function (Duration ?t)
             (Traj ?t))
)