(define (problem pb2)
   (:domain blocksworld)
   (:objects a b)
   (:init
     (on-table a)
     (on b a)
     (clear b)
     (arm-empty))
   (:goal (on a b)))