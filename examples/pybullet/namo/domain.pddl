(define (domain rovers)
  (:requirements :strips :equality)
  (:predicates
    (Robot ?r)
    (Conf ?r ?q)
    (Motion ?r ?q1 ?q2 ?t)

    (AtConf ?r ?q)
  )

  (:action move
    :parameters (?r ?q1 ?q2 ?t)
    :precondition (and (Motion ?r ?q1 ?q2 ?t)
                       (AtConf ?r ?q1))
    :effect (and (AtConf ?r ?q2)
                 (not (AtConf ?r ?q1)))
  )

)