(define (problem strips-sat-x-1)
(:domain satellite)
(:objects
	satellite0
	instrument0
	image1
	spectrograph2
	thermograph0
	Star0
	GroundStation1
	GroundStation2
	Phenomenon3
	Phenomenon4
	Star5
	Phenomenon6
)
(:init
	(satellite satellite0)
	(instrument instrument0)
	(supports instrument0 thermograph0)
	(calibration_target instrument0 GroundStation2)
	(on_board instrument0 satellite0)
	(power_avail satellite0)
	(pointing satellite0 Phenomenon6)
	(mode image1)
	(mode spectrograph2)
	(mode thermograph0)
	(direction Star0)
	(direction GroundStation1)
	(direction GroundStation2)
	(direction Phenomenon3)
	(direction Phenomenon4)
	(direction Star5)
	(direction Phenomenon6)
)
(:goal (and
	(have_image Phenomenon4 thermograph0)
	(have_image Star5 thermograph0)
	(have_image Phenomenon6 thermograph0)
))

)