(define (domain kitchen2d)
    (:requirements :strips :equality)
    (:predicates
    	; Static predicates (predicates that do not change over time)
    	(IsGripper ?gripper)
    	(IsCup ?cup)
    	(IsStirrer ?kettle) ;kettle is both spoon and stirrer
    	(IsSpoon ?kettle)
    	(IsBlock ?cup)	;why is cup a block
    	(IsPourable ?cup)

    	(IsPose ?cup ?pose) ;only cup has pose?
    	(IsGrasp ?cup ?grasp)
    	(IsControl ?control)

    	(CanGrasp ?gripper ?pose ?cup ?pose2 ?grasp ?control)
    	(BelowFaucet ?gripper ?pose ?cup ?grasp)
    	(CanPour ?gripper ?pose ?cup ?grasp ?kettle ?pose2 ?control)
    	(Motion ?gripper ?pose ?pose2 ?control)
    	(MotionH ?gripper ?pose ?cup ?grasp ?pose2 ?control)

    	(CanScoop ?gripper ?pose ?pose2 ?spoon ?grasp ?kettle ?pose3 ?control)
    	(CanDump ?gripper ?pose ?pose3 ?spoon ?grasp ?kettle ?pose2 ?control)
    	(CanStir ?gripper ?pose ?spoon ?grasp ?kettle ?pose2 ?control)
    	(CanPush ?gripper ?pose ?pose2 ?cup ?pose3 ?pose4 ?control)

    	(Stackable ?cup ?block)
    	(BlockSupport ?cup ?pose ?block ?pose2)
    	(Clear ?block)
    	(TableSupport ?pose)

    	; Fluent predicates (predicates that change over time, which describes the state of the sytem)
    	(AtPose ?cup ?block)
    	(Grasped ?cup ?grasp)
    	(Empty ?gripper)
    	(CanMove ?gripper)
    	(HasCoffee ?cup)
    	(HasSugar ?cup)
    	(HasCream ?cup)
    	(Mixed ?cup)
    	(Scooped ?cup)

    	; Derived predicates (predicates derived from other predicates, defined with streams)
    	(Unsafe ?control)
    	(Holding ?cup)
    	(On ?cup ?block)

    	; External predicates (evaluated by external boolean functions)
    	(Collision ?control ?gripper ?pose)
    )

    (:action move
    	:parameters (?gripper ?pose ?pose2 ?control)
    	:precondition
    		(and (Motion ?gripper ?pose ?pose2 ?control)
    			 ; (CanMove ?gripper) ; (Empty ?gripper)
    			 (AtPose ?gripper ?pose) (not (Unsafe ?control)))
    	:effect
    		(and (AtPose ?gripper ?pose2)
    			 (not (AtPose ?gripper ?pose))
    			 (not (CanMove ?gripper))
    			 (increase (total-cost) 1))
    )
    ;(:action move-holding
    ;	:parameters (?gripper ?pose ?cup ?grasp ?pose2 ?control)
    ;	:precondition
    ;		(and (MotionH ?gripper ?pose ?cup ?grasp ?pose2 ?control)
    ;			 (not (Empty ?gripper)) (AtPose ?gripper ?pose)
    ;		     (Grasped ?cup ?grasp) (CanMove ?gripper) (not (Unsafe ?control)))
    ;	:effect
    ;		(and (AtPose ?gripper ?pose2) (not (AtPose ?gripper ?pose))
    ;			 (not (CanMove ?gripper))
    ;            (increase (total-cost) 1))
    ;)
    (:action push
    	:parameters (?gripper ?pose ?pose2 ?block ?pose3 ?pose4 ?control)
    	:precondition
    		(and (CanPush ?gripper ?pose ?pose2 ?block ?pose3 ?pose4 ?control)
    			 (AtPose ?gripper ?pose) (AtPose ?block ?pose3)
    			 (Empty ?gripper) (Clear ?block))
    	:effect
    		(and (AtPose ?gripper ?pose2) (AtPose ?block ?pose4)
    			 (CanMove ?gripper) (not (AtPose ?gripper ?pose))
    			 (not (AtPose ?block ?pose3))
    			 (increase (total-cost) 1))
    )
    (:action pick
    	:parameters	(?gripper ?pose ?cup ?pose2 ?grasp ?control)
    	:precondition
    		(and (CanGrasp ?gripper ?pose ?cup ?pose2 ?grasp ?control)
    			 (AtPose ?gripper ?pose) (AtPose ?cup ?pose2)
    			 (Empty ?gripper) (TableSupport ?pose2))
    	:effect
    		(and (Grasped ?cup ?grasp) (CanMove ?gripper)
    			 (not (AtPose ?cup ?pose2)) (not (Empty ?gripper))
    			 (increase (total-cost) 1))
    )
    (:action place
		:parameters (?gripper ?pose ?cup ?pose2 ?grasp ?control)
		:precondition
			(and (CanGrasp ?gripper ?pose ?cup ?pose2 ?grasp ?control)
				 (AtPose ?gripper ?pose) (TableSupport ?pose2)
				 (Grasped ?cup ?grasp) (not (Scooped ?cup)))
		:effect
			(and (AtPose ?cup ?pose2) (Empty ?gripper)
				 (CanMove ?gripper) (not (Grasped ?cup ?grasp))
				 (increase (total-cost) 1))
    )
    (:action stack
    	:parameters (?gripper ?pose ?cup ?pose2 ?grasp ?block ?pose3 ?control)
    	:precondition
    		(and (CanGrasp ?gripper ?pose ?cup ?pose2 ?grasp ?control)
    			 (BlockSupport ?cup ?pose2 ?block ?pose3)
    			 (AtPose ?gripper ?pose) (Grasped ?cup ?grasp)
    			 (AtPose ?block ?pose3) (Clear ?block))
    	:effect
    		(and (AtPose ?cup ?pose2) (Empty ?gripper)
    			 (CanMove ?gripper) (not (Grasped ?cup ?grasp))
    			 (not (Clear ?block))
    			 (increase (total-cost) 1))
    )
    (:action fill
    	:parameters (?gripper ?pose ?cup ?grasp)
    	:precondition
    		(and (BelowFaucet ?gripper ?pose ?cup ?grasp)
    		     (AtPose ?gripper ?pose) (Grasped ?cup ?grasp))
    	:effect
    		(and (HasCoffee ?cup) (CanMove ?gripper)
    			 (increase (total-cost) 1))
    )
    (:action pour
    	:parameters (?gripper ?pose ?cup ?grasp ?kettle ?pose2 ?control)
    	:precondition
    		(and (CanPour ?gripper ?pose ?cup ?grasp ?kettle ?pose2 ?control)
    			 (AtPose ?gripper ?pose) (Grasped ?cup ?grasp)
    			 (AtPose ?kettle ?pose2) (HasCream ?cup))

    	:effect
    		(and (HasCream ?kettle) (CanMove ?gripper)
    			 (not (HasCream ?cup))
    			 (increase (total-cost) 1))
    )
    (:action scoop
    	:parameters (?gripper ?pose ?pose2 ?spoon ?grasp ?kettle ?pose3 ?control)
    	:precondition
    		(and (CanScoop ?gripper ?pose ?pose2 ?spoon ?grasp ?kettle ?pose3 ?control)
    			 (AtPose ?gripper ?pose) (Grasped ?spoon ?grasp)
    			 (AtPose ?kettle ?pose3) (HasSugar ?kettle))
    	:effect
    		(and (AtPose ?gripper ?pose2) (HasSugar ?spoon)
    			 (CanMove ?gripper) (Scooped ?spoon)
    			 (not (AtPose ?gripper ?pose))
    			 (increase (total-cost) 1))
    )
    (:action dump
    	:parameters (?gripper ?pose ?pose3 ?spoon ?grasp ?kettle ?pose2 ?control)
    	:precondition
    		(and (CanDump ?gripper ?pose ?pose3 ?spoon ?grasp ?kettle ?pose2 ?control)
    			 (AtPose ?gripper ?pose) (Grasped ?spoon ?grasp)
    			 (AtPose ?kettle ?pose2) (HasSugar ?spoon))
    	:effect
    		(and (HasSugar ?kettle) (CanMove ?gripper)
    			 (not (HasSugar ?spoon)) (not (Scooped ?spoon))
    			 (not (AtPose ?gripper ?pose)) (AtPose ?gripper ?pose3)
    			 (increase (total-cost) 1))
    )
    (:action stir
    	:parameters (?gripper ?pose ?spoon ?grasp ?kettle ?pose2 ?control)
    	:precondition
    		(and (CanStir ?gripper ?pose ?spoon ?grasp ?kettle ?pose2 ?control)
    			 (AtPose ?gripper ?pose) (Grasped ?spoon ?grasp)
    			 (AtPose ?kettle ?pose2) (HasCoffee ?kettle)
    			 (HasCream ?kettle) (HasSugar ?kettle))
    	:effect
    		(and (Mixed ?kettle) (CanMove ?gripper)
    			 (increase (total-cost) 1))
    )

    ;(:derived (Unsafe ?control)
    ;    (exists (?cup ?pose) (and (Collision ?control ?cup ?pose) (AtPose ?cup ?pose)))
    ;)
    (:derived (Holding ?cup)
        (exists (?grasp) (and (IsGrasp ?cup ?grasp)
                              (Grasped ?cup ?grasp)))
    )
    (:derived (On ?cup ?block)
        (exists (?pose ?pose2) (and (BlockSupport ?cup ?pose ?block ?pose2)
                                    (AtPose ?cup ?pose) (AtPose ?block ?pose2)))
    )
)

