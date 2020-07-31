;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 4 op-blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain rohanblocks)
    (:requirements :strips :typing)
    (:predicates 
        (on ?x ?y)
        (ontable ?x)
        (clear ?x)
        (handempty ?robot)
        (handfull ?robot)
        (holding ?x)
        (isblock ?x)
        (isrobot ?robot)
        (pickup ?x)
        (putdown ?x)
        (stack ?x ?y)
        (unstack ?x)
    )

    ; (:actions pickup putdown stack unstack)

    (:action pick-up
        :parameters (?x ?robot)
        :precondition (and
            (pickup ?x) 
            (clear ?x) 
            (ontable ?x) 
            (handempty ?robot)
            (isblock ?x)
            (isrobot ?robot)
        )
        :effect (and
            (not (ontable ?x))
            (not (clear ?x))
            (not (handempty ?robot))
            (handfull ?robot)
            (holding ?x)
        )
    )

    (:action put-down
        :parameters (?x ?robot)
        :precondition (and 
            (putdown ?x)
            (holding ?x)
            (handfull ?robot)
            (isblock ?x)
            (isrobot ?robot)
        )
        :effect (and 
            (not (holding ?x))
            (clear ?x)
            (handempty ?robot)
            (not (handfull ?robot))
            (ontable ?x))
        )

    (:action stack
        :parameters (?x ?y ?robot)
        :precondition (and
            (stack ?x ?y)
            (holding ?x) 
            (clear ?y)
            (handfull ?robot)
            (isblock ?x)
            (isblock ?y)
            (isrobot ?robot)
        )
        :effect (and 
            (not (holding ?x))
            (not (clear ?y))
            (clear ?x)
            (handempty ?robot)
            (not (handfull ?robot))
            (on ?x ?y)
        )
    )

    (:action unstack
        :parameters (?x ?y ?robot)
        :precondition (and
            (unstack ?x)
            (on ?x ?y)
            (clear ?x)
            (handempty ?robot)
            (isblock ?x)
            (isblock ?y)
            (isrobot ?robot)
        )
        :effect (and 
            (holding ?x)
            (clear ?y)
            (not (clear ?x))
            (not (handempty ?robot))
            (handfull ?robot)
            (not (on ?x ?y))
        )
    )
)
