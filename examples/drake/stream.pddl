(define (stream kuka-tamp)
  (:stream sample-pose
    :inputs (?o ?s)
    :domain (Stackable ?o ?s)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?s))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )
  (:stream inverse-kinematics
    :inputs (?o ?p ?g)
    :domain (and (Pose ?o ?p) (Grasp ?o ?g))
    :outputs (?q ?t)
    :certified (and (Conf ?q) (Traj ?t) (Kin ?o ?p ?g ?q ?t) (GraspConf ?o ?g ?q)) ; TODO: (GraspConf ?o ?g ?q) for initial conf
  )
  (:stream plan-free-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    ;:fluents (AtPose)
    :outputs (?t)
    ;:certified (and (Traj ?t) (FreeMotion ?q1 ?t ?q2))
    :certified (FreeMotion ?q1 ?t ?q2)
  )
  (:stream plan-holding-motion
    :inputs (?q1 ?q2 ?o ?g)
    ;:domain (and (Conf ?q1) (Conf ?q2) (Grasp ?o ?g))
    :domain (and (GraspConf ?o ?g ?q1) (GraspConf ?o ?g ?q2)) ; Condition that ?o ?g make sense for ?q1 ?q2
    ;:fluents (AtPose)
    :outputs (?t)
    ;:certified (and (Traj ?t) (HoldingMotion ?q1 ?t ?q2 ?o ?g))
    :certified (HoldingMotion ?q1 ?t ?q2 ?o ?g)
  )

  ;(:predicate (TrajCollision ?t ?o2 ?p2)
  ;  (and (Traj ?t) (Pose ?o2 ?p2))
  ;)
)