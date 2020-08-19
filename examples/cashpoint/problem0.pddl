(define (problem nightout)
 (:domain nightout)
 (:objects Emre - person
           atm1 atm2 atm3 - machine)
 (:init
    (= (minwithdraw atm1) 5)
    (= (maxwithdraw atm1) 30)

    (= (minwithdraw atm2) 5)
    (= (maxwithdraw atm2) 30)

    (= (minwithdraw atm3) 5)
    (= (maxwithdraw atm3) 30)

    (= (inpocket Emre) 2)
 )
 (:goal (and (finished)))
 (:metric minimize (total-time))
)