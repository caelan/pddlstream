(define (stream kuka-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Supported ?o ?p ?r)
                    (Pose ?o ?p) (Observable ?p))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )

  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g));  (Observable ?p))
    :outputs (?bq ?t)
    :certified (and (Kin ?a ?o ?p ?g ?bq ?t)
                    (BConf ?bq) (Traj ?a ?t)
                    (RegRange ?o ?p ?bq) (VisRange ?o ?p ?bq))
  )
  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    :outputs (?t)
    :certified (and (BTraj ?t)
                    (BaseMotion ?q1 ?t ?q2))
  )

  ; Alternatively, could just do inverse visibility
  (:stream test-vis-base
    :inputs (?o ?p ?bq)
    :domain (and (Pose ?o ?p) (BConf ?bq))
    :outputs ()
    :certified (VisRange ?o ?p ?bq)
  )
  (:stream test-reg-base
    :inputs (?o ?p ?bq)
    :domain (and (Pose ?o ?p) (BConf ?bq))
    :outputs ()
    :certified (and (RegRange ?o ?p ?bq) (VisRange ?o ?p ?bq))
  )

  (:stream sample-vis-base
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq)
    :certified (VisRange ?o ?p ?bq)
  )
  (:stream sample-reg-base
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq)
    :certified (and (VisRange ?o ?p ?bq) (RegRange ?o ?p ?bq))
  )
  (:stream inverse-visibility
    :inputs (?o ?p ?bq)
    :domain (VisRange ?o ?p ?bq)
    :outputs (?hq ?ht)
    :certified (and (Vis ?o ?p ?bq ?hq ?ht) ; Only set BConf on last iteration
                    (BConf ?bq) (Conf head ?hq) (Traj head ?ht))
  )
)