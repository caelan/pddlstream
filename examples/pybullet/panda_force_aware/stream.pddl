(define (stream bi-panda-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r) (ForcesBalanced ?o ?p))
  )

  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )

  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g))
    :outputs (?q ?t)
    :certified (and (BConf ?q) (ATraj ?t) (Kin ?a ?o ?p ?g ?q ?t) (TorqueLimitsNotExceded ?a)) ; (ATraj ?t)
  )

  (:stream test_torque_limits_not_exceded
   :inputs (?a)
   :domain (and (Arm ?a))
   :certified (TorqueLimitsNotExceded ?a)
  )

  (:stream test-cfree-pose-pose
    :inputs (?o1 ?p1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Pose ?o2 ?p2))
    :certified (CFreePosePose ?o1 ?p1 ?o2 ?p2)
  )
  (:stream test-cfree-approach-pose
    :inputs (?o1 ?p1 ?g1 ?o2 ?p2)
    :domain (and (Pose ?o1 ?p1) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
    :certified (CFreeApproachPose ?o1 ?p1 ?g1 ?o2 ?p2)
  )

  (:stream test-cfree-traj-pose
    :inputs (?t ?o2 ?p2)
    :domain (and (ATraj ?t) (Pose ?o2 ?p2))
    :certified (CFreeTrajPose ?t ?o2 ?p2)
  )


  ;(:stream test-cfree-traj-grasp-pose
  ;  :inputs (?t ?a ?o1 ?g1 ?o2 ?p2)
  ;  :domain (and (BTraj ?t) (Arm ?a) (Grasp ?o1 ?g1) (Pose ?o2 ?p2))
  ;  :certified (CFreeTrajGraspPose ?t ?a ?o1 ?g1 ?o2 ?p2)
  ;)


  ;(:function (MoveCost ?t)
  ;  (and (BTraj ?t))
  ;)
)