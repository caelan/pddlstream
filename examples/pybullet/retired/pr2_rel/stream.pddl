(define (stream kuka-tamp)
  (:function (Distance ?q1 ?q2)
    (and (BConf ?q1) (BConf ?q2))
  )

  (:predicate (PoseCollision ?b1 ?p1 ?b2 ?p2)
    (and (WorldPose ?b1 ?p1) (WorldPose ?b2 ?p2) (Graspable ?b1) (Graspable ?b2))
  )

  (:stream sample-pose ; TODO: RelPose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (RelPose ?o ?p ?r)
  )
  (:stream sample-grasp
    :inputs (?a ?o)
    :domain (and (Controllable ?a) (Graspable ?o))
    :outputs (?g)
    :certified (Grasp ?a ?o ?g)
  )
  (:stream inverse-kinematics
    :inputs (?o ?p ?g ?a)
    :domain (and (WorldPose ?o ?p) (Grasp ?a ?o ?g))
    :outputs (?bq ?aq)
    :certified (and (BConf ?bq) (AConf ?a ?aq) (Kin ?a ?o ?p ?g ?bq ?aq))
  )

  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    :outputs (?t)
    :certified (BaseMotion ?q1 ?t ?q2)
  )
  (:stream plan-pick-motion
    :inputs (?a ?o ?p ?g ?bq ?aq)
    :domain (Kin ?a ?o ?p ?g ?bq ?aq)
    :outputs (?t)
    :certified (PickMotion ?a ?o ?p ?g ?bq ?aq ?t)
  )
  (:stream plan-place-motion
    :inputs (?a ?o ?p ?g ?bq ?aq)
    :domain (Kin ?a ?o ?p ?g ?bq ?aq)
    :outputs (?t)
    :certified (PlaceMotion ?a ?o ?p ?g ?bq ?aq ?t)
  )

  (:stream compose-poses
    :inputs (?o1 ?p1 ?o2 ?rp)
    :domain (and (WorldPose ?o1 ?p1) (RelPose ?o2 ?rp ?o1))
    :outputs (?p2)
    :certified (and (WorldPose ?o2 ?p2) (TForm ?o2 ?p2 ?rp ?o1 ?p1))
  )

  ;(:stream plan-arm-motion
  ;  :inputs (?a ?q1 ?q2)
  ;  :domain (and (AConf ?a ?q1) (AConf ?a ?q2))
  ;  :outputs (?t)
  ;  :certified (and (BTraj ?t) (ArmMotion ?a ?q1 ?t ?q2))
  ;)
)