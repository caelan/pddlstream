(define (stream continuous-tamp)
  (:stream s-region
    :inputs (?b ?r)
    :domain (Placeable ?b ?r)
    :outputs (?p)
    :certified (and (Pose ?b ?p) (Contain ?b ?p ?r)))

  (:stream s-ik
    :inputs (?b ?p ?g)
    :domain (and (Pose ?b ?p) (Grasp ?b ?g))
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?b ?q ?p ?g)))

  (:stream connect
    :inputs (?q)
    :domain (Conf ?q))

  (:stream t-cfree
    :inputs (?t ?b1 ?g1 ?b2 ?p2)
    :domain (and (Traj ?t) (Grasp ?b1 ?g1) (Pose ?b2 ?p2))
    :certified (CFree ?t ?b1 ?g1 ?b2 ?p2))

  (:stream t-region
    :inputs (?b ?p ?r)
    :domain (and (Pose ?b ?p) (Placeable ?b ?r))
    :certified (Contain ?b ?p ?r))
)