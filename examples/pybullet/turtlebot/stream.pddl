(define (stream turtlebot)
  (:stream compute-motion
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :outputs (?t)
    :certified (and (Traj ?t) (Motion ?q1 ?t ?q2)))

  (:stream test-cfree-conf-conf
    :inputs (?q1 ?q2)
    :domain (and (Conf ?q1) (Conf ?q2))
    :certified (CFreeConfConf ?q1 ?q2))
  (:stream test-cfree-traj-conf
    :inputs (?t1 ?q2)
    :domain (and (Traj ?t1) (Conf ?q2))
    :certified (CFreeTrajConf ?t1 ?q2))
  (:stream test-cfree-traj-traj
    :inputs (?t1 ?t2)
    :domain (and (Traj ?t1) (Traj ?t2))
    :certified (CFreeTrajTraj ?t1 ?t2))

  (:function (TrajDistance ?t)
     (Traj ?t))
)