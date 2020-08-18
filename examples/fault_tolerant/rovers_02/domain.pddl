(define (domain rover)
(:requirements :strips)
(:predicates
	 (at ?x ?y) (at_lander ?x ?y) (can_traverse ?r ?x ?y) (equipped_for_soil_analysis ?r) (equipped_for_rock_analysis ?r) (equipped_for_imaging ?r) (empty ?s) (have_rock_analysis ?r ?w) (have_soil_analysis ?r ?w) (full ?s) (calibrated ?c ?r) (supports ?c ?m) (available ?r) (visible ?w ?p) (have_image ?r ?o ?m) (communicated_soil_data ?w) (communicated_rock_data ?w) (communicated_image_data ?o ?m) (at_soil_sample ?w) (at_rock_sample ?w) (visible_from ?o ?w) (store_of ?s ?r) (calibration_target ?i ?o) (on_board ?i ?r) (channel_free ?l)(rover ?x) (waypoint ?x) (store ?x) (camera ?x) (mode ?x) (lander ?x) (objective ?x) )
(:action navigate
 :parameters ( ?x ?y ?z)
 :precondition
	(and (rover ?x) (waypoint ?y) (waypoint ?z)  (can_traverse ?x ?y ?z) (available ?x) (at ?x ?y) (visible ?y ?z))
 :effect
	(and (at ?x ?z) (not (at ?x ?y))))

(:action sample_soil
 :parameters ( ?x ?s ?p)
 :precondition
	(and (rover ?x) (store ?s) (waypoint ?p)  (at ?x ?p) (at_soil_sample ?p) (equipped_for_soil_analysis ?x) (store_of ?s ?x) (empty ?s))
 :effect
	(and (full ?s) (have_soil_analysis ?x ?p) (not (empty ?s)) (not (at_soil_sample ?p))))

(:action sample_rock
 :parameters ( ?x ?s ?p)
 :precondition
	(and (rover ?x) (store ?s) (waypoint ?p) (at ?x ?p) (at_rock_sample ?p) (equipped_for_rock_analysis ?x) (store_of ?s ?x) (empty ?s))
 :effect
	(and (full ?s) (have_rock_analysis ?x ?p) (not (empty ?s)) (not (at_rock_sample ?p))))

(:action drop
 :parameters ( ?x ?y)
 :precondition
	(and (rover ?x) (store ?y)  (store_of ?y ?x) (full ?y))
 :effect
	(and (empty ?y) (not (full ?y))))

(:action calibrate
 :parameters ( ?r ?i ?t ?w)
 :precondition
	(and (rover ?r) (camera ?i) (objective ?t) (waypoint ?w)  (equipped_for_imaging ?r) (calibration_target ?i ?t) (at ?r ?w) (visible_from ?t ?w) (on_board ?i ?r))
 :effect
	 (calibrated ?i ?r))

(:action take_image
 :parameters ( ?r ?p ?o ?i ?m)
 :precondition
	(and (rover ?r) (waypoint ?p) (objective ?o) (camera ?i) (mode ?m)  (calibrated ?i ?r) (on_board ?i ?r) (equipped_for_imaging ?r) (supports ?i ?m) (visible_from ?o ?p) (at ?r ?p))
 :effect
	(and (have_image ?r ?o ?m) (not (calibrated ?i ?r))))

(:action communicate_soil_data
 :parameters ( ?r ?l ?p ?x ?y)
 :precondition
	(and (rover ?r) (lander ?l) (waypoint ?p) (waypoint ?x) (waypoint ?y)  (at ?r ?x) (at_lander ?l ?y) (have_soil_analysis ?r ?p) (visible ?x ?y) (available ?r) (channel_free ?l))
 :effect
	(and (channel_free ?l) (communicated_soil_data ?p) (available ?r) (not (available ?r)) (not (channel_free ?l))))

(:action communicate_rock_data
 :parameters ( ?r ?l ?p ?x ?y)
 :precondition
	(and (rover ?r) (lander ?l) (waypoint ?p) (waypoint ?x) (waypoint ?y)  (at ?r ?x) (at_lander ?l ?y) (have_rock_analysis ?r ?p) (visible ?x ?y) (available ?r) (channel_free ?l))
 :effect
	(and (channel_free ?l) (communicated_rock_data ?p) (available ?r) (not (available ?r)) (not (channel_free ?l))))

(:action communicate_image_data
 :parameters ( ?r ?l ?o ?m ?x ?y)
 :precondition
	(and (rover ?r) (lander ?l) (objective ?o) (mode ?m) (waypoint ?x) (waypoint ?y)  (at ?r ?x) (at_lander ?l ?y) (have_image ?r ?o ?m) (visible ?x ?y) (available ?r) (channel_free ?l))
 :effect
	(and (channel_free ?l) (communicated_image_data ?o ?m) (available ?r) (not (available ?r)) (not (channel_free ?l))))

)
