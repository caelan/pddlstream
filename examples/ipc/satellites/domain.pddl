(define (domain satellite)
(:requirements :equality :strips)
(:predicates
	 (on_board ?i ?s) (supports ?i ?m) (pointing ?s ?d) (power_avail ?s) (power_on ?i) (calibrated ?i) (have_image ?d ?m) (calibration_target ?i ?d)(satellite ?x) (direction ?x) (instrument ?x) (mode ?x) )
(:action turn_to
 :parameters ( ?s ?d_new ?d_prev)
 :precondition
	(and (satellite ?s) (direction ?d_new) (direction ?d_prev)  (pointing ?s ?d_prev))
 :effect
	(and (pointing ?s ?d_new) (not (pointing ?s ?d_prev))))

(:action switch_on
 :parameters ( ?i ?s)
 :precondition
	(and (instrument ?i) (satellite ?s)  (on_board ?i ?s) (power_avail ?s))
 :effect
	(and (power_on ?i) (not (calibrated ?i)) (not (power_avail ?s))))

(:action switch_off
 :parameters ( ?i ?s)
 :precondition
	(and (instrument ?i) (satellite ?s)  (on_board ?i ?s) (power_on ?i))
 :effect
	(and (power_avail ?s) (not (power_on ?i))))

(:action calibrate
 :parameters ( ?s ?i ?d)
 :precondition
	(and (satellite ?s) (instrument ?i) (direction ?d)  (on_board ?i ?s) (calibration_target ?i ?d) (pointing ?s ?d) (power_on ?i))
 :effect
	 (calibrated ?i))

(:action take_image
 :parameters ( ?s ?d ?i ?m)
 :precondition
	(and (satellite ?s) (direction ?d) (instrument ?i) (mode ?m)  (calibrated ?i) (on_board ?i ?s) (supports ?i ?m) (power_on ?i) (pointing ?s ?d) (power_on ?i))
 :effect
	 (have_image ?d ?m))

)