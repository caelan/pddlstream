(define (stream kuka-tamp)
  (:stream sample-pose
    :inputs (?o ?r)
    :domain (Stackable ?o ?r)
    :outputs (?p)
    :certified (and (Pose ?o ?p) (Supported ?o ?p ?r) (Observable ?p))
  )
  (:stream sample-grasp
    :inputs (?o)
    :domain (Graspable ?o)
    :outputs (?g)
    :certified (Grasp ?o ?g)
  )
  ; TODO: only set (BConf ?q1) on the final step for movements

  (:stream inverse-kinematics
    :inputs (?a ?o ?p ?g)
    :domain (and (Controllable ?a) (Pose ?o ?p) (Grasp ?o ?g));  (Observable ?p))
    :outputs (?bq ?t)
    :certified (and (BConf ?bq) (Traj ?a ?t) (ScanRange ?o ?p ?bq) ; (LookRange ?o ?p ?bq)
                    (Kin ?a ?o ?p ?g ?bq ?t))
  )
  (:stream plan-base-motion
    :inputs (?q1 ?q2)
    :domain (and (BConf ?q1) (BConf ?q2))
    :outputs (?t)
    :certified (and (BTraj ?t)
                    (BaseMotion ?q1 ?t ?q2))
  )
  (:stream inverse-visibility
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq ?hq)
    :certified (and (BConf ?bq) (Conf head ?hq) (LookRange ?o ?p ?bq)
                    (Vis ?o ?p ?bq ?hq))
  )
  (:stream plan-scan
    :inputs (?o ?p)
    :domain (Pose ?o ?p)
    :outputs (?bq ?hq ?ht)
    :certified (and (BConf ?bq) (Conf head ?hq) (Traj head ?ht) (ScanRange ?o ?p ?bq)
                    (Scan ?o ?p ?bq ?hq ?ht))
  )
)