(define (stream pick-and-place)
  (:function (Dist ?q1 ?q2)
    (and (Conf ?q1) (Conf ?q2))
  )
  ;(:predicate (TrajCollision ?t ?b2 ?p2)
  ;  (and (Traj ?t) (Pose ?b2 ?p2))
  ;)

  ; Creates fewer free variables than optimizer.pddl
  (:optimizer gurobi

    ; Constructs a set of free variables
    ;(:variables (?p)
    ;  :inputs (?b) ; TODO: input-domain, variable-domain, codomain, image
    ;  :domain (Block ?b)
    ;  :graph (Pose ?b ?p))
    (:variables (?p)
      :inputs (?b ?r)
      :domain (Placeable ?b ?r)
      :graph (and (Contain ?b ?p ?r) (Pose ?b ?p)))
    (:variables (?q)
      :graph (Conf ?q))

    ; Constraint forms that can be optimized
    ; TODO: can fix variables in the optimization using necessary conditions
    ;(:constraint (Contain?b ?p ?r) ; TODO: make this a cluster of constraints?
    ; :necessary (and (Placeable ?b ?r) (Pose ?b ?p)))
    (:constraint (Kin ?b ?q ?p ?g)
      :necessary (and (Pose ?b ?p) (Grasp ?b ?g) (Conf ?q)))
    (:constraint (CFree ?b1 ?p1 ?b2 ?p2)
      :necessary (and (Pose ?b1 ?p1) (Pose ?b2 ?p2)))
    ; TODO: maybe prevent initial configurations from being considered

    ; Additive objective functions
    (:objective Dist)
  )

  ;(:optimizer rrt
  ;  (:variables (?t)
  ;    :graph (Traj ?t))
  ;  (:constraint (Motion ?q1 ?t ?q2)
  ;    :necessary (and (Conf ?q1) (Traj ?t) (Conf ?q2)));
  ;
  ;  ; Treating predicate as objective
  ;  ;(:objective TrajCollision)
  ;)

  (:optimizer rrt
    (:variables (?t)
      :inputs (?q1 ?q2)
      :domain (and (Conf ?q1) (Conf ?q2))
      :graph (and (Motion ?q1 ?t ?q2) (Traj ?t)))
  )
)