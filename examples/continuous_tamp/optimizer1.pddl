(define (stream continuous-tamp)
  (:function (Distance ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )
  ;(:predicate (PoseCollision ?b1 ?p1 ?b2 ?p2)
  ;  (and (Pose ?b1 ?p1) (Pose ?b2 ?p2))
  ;)
  ;(:predicate (TrajCollision ?t ?b2 ?p2)
  ;  (and (Traj ?t) (Pose ?b2 ?p2))
  ;)

  ; Creates more free variables than optimizer2.pddl
  (:optimizer gurobi

    ; Constructs a set of free variables
    (:variable ?p
      :inputs (?b ?r)
      :domain (Placeable ?b ?r)
      :graph (and (Contained ?b ?p ?r) (Pose ?b ?p)))
    (:variable ?q
      :inputs (?b ?p)
      :domain (Pose ?b ?p)
      :graph (and (Kin ?b ?q ?p) (Conf ?q)))
    (:constraint (CFree ?b1 ?p1 ?b2 ?p2)
      :necessary (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)))

    ; Treating predicates as objectives
    ;(:objective PoseCollision)
    (:objective Distance)
  )

  (:optimizer rrt
    (:variable ?t
      :inputs (?q1 ?q2)
      :domain (and (Conf ?q1) (Conf ?q2))
      :graph (and (Motion ?q1 ?t ?q2) (Traj ?t)))

    ; Treating predicate as an objective
    ;(:objective TrajCollision)
  )
)