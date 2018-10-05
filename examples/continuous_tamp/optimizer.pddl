(define (stream pick-and-place)
  (:function (Distance ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )
  ;(:predicate (PoseCollision ?b1 ?p1 ?b2 ?p2)
  ;  (and (Pose ?b1 ?p1) (Pose ?b2 ?p2))
  ;)
  ;(:predicate (TrajCollision ?t ?b2 ?p2)
  ;  (and (Traj ?t) (Pose ?b2 ?p2))
  ;)

  ; This representation is nice because no need to specify free variables
  (:optimizer gurobi

    ; Constructs a set of free variables
    ;(:variable ?p
    ;  :inputs (?b) ; TODO: input-domain, variable-domain, codomain, image
    ;  :domain (Block ?b)
    ;  :graph (Pose ?b ?p))
    (:variable ?p
      :inputs (?b ?r)
      :domain (Placeable ?b ?r)
      :graph (and (Contained ?b ?p ?r) (Pose ?b ?p)))

    (:variable ?q
      :graph (Conf ?q))
    ;(:variable ?q
    ;  :inputs (?b ?p)
    ;  :domain (Pose ?b ?p)
    ;  :graph (and (Kin ?b ?q ?p) (Conf ?q)))

    ; TODO: can ensure that no fixed things are optimized by making conditions involve just variable
    ; Constraint forms that can be optimized
    ;(:constraint (Contained ?b ?p ?r) ; TODO: make this a cluster of constraints?
    ; :necessary (and (Placeable ?b ?r) (Pose ?b ?p)))
    (:constraint (Kin ?b ?q ?p)
     :necessary (and (Pose ?b ?p) (Conf ?q)))
    ;(:constraint (SafePose ?b ?p) ; Semantics are that the subset of the state is an input (functional STRIPS)
    ; :fluents (AtPose)
    ; :necessary (and (Pose ?b ?p)))
    ;(:constraint (CFree ?b1 ?p1 ?b2 ?p2)
    ; :necessary (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)))

    (:objective PoseCollision) ; Treating predicate as objective
    (:objective Distance)
  )

  (:optimizer rrt
    ;(:variable ?t
    ;  :graph (Traj ?t))
    (:variable ?t
      :inputs (?q1 ?q2)
      :domain (and (Conf ?q1) (Conf ?q2))
      :graph (and (Motion ?q1 ?t ?q2) (Traj ?t)))

    ;(:constraint (Motion ?q1 ?t ?q2)
    ; :necessary (and (Conf ?q1) (Traj ?t) (Conf ?q2)))

    (:objective TrajCollision) ; Treating predicate as objective
  )
)