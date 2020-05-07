(define (stream continuous-tamp)
  (:stream s-region
    :inputs (?b ?r)
    :domain (Placeable ?b ?r)
    :outputs (?p)
    :certified (and (Pose ?b ?p) (Contain ?b ?p ?r)))
  ; TODO: s-grasp

  (:stream s-ik
    :inputs (?b ?p ?g)
    :domain (and (Pose ?b ?p) (Grasp ?b ?g))
    :outputs (?q)
    :certified (and (Conf ?q) (Kin ?b ?q ?p ?g)))

  (:stream s-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :fluents (AtPose AtGrasp) ; TODO: AtConf for other robots
    :outputs (?t)
    :certified (and (Motion ?q1 ?t ?q2))) ; (Reachable ?q1 ?q2)

  ;(:stream t-reachable
  ;  :inputs (?q1 ?q2)
  ;  :domain (and (Conf ?q1) (Conf ?q2))
  ;  :fluents (AtPose AtGrasp) ; TODO: AtConf for other robots
  ;  :certified (Reachable ?q1 ?q2))

  ;(:stream t-region
  ;  :inputs (?b ?p ?r)
  ;  :domain (and (Pose ?b ?p) (Placeable ?b ?r))
  ;  :certified (Contain ?b ?p ?r))

  ;(:function (Dist ?q1 ?q2)
  ;  (and (Conf ?q1) (Conf ?q2))
  ;)
)