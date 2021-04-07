(define (stream rovers)

  (:stream test-reachable ; Eager
    :inputs (?v ?bq)
    :domain (and (Rover ?v) (BConf ?bq))
    :certified (Reachable ?v ?bq)
  )
  ;(:stream obj-range ; Eager
  ;  :inputs (?bq ?o)
  ;  :domain (and (BConf ?bq) (Objective ?o))
  ;  :certified (ImageRange ?bq ?o)
  ;)
  ;(:stream com-range ; Eager
  ;  :inputs (?bq ?l)
  ;  :domain (and (BConf ?bq) (Lander ?l))
  ;  :certified (ComRange ?bq ?l)
  ;)
  (:stream obj-inv-visible
    :inputs (?o)
    :domain (and (Objective ?o))
    :outputs (?bq ?hq ?y)
    :certified (and (ImageVisible ?bq ?hq ?y ?o) (ImageRange ?bq ?o)
                    (BConf ?bq) (HConf ?hq) (Ray ?y))
  )
  (:stream com-inv-visible
    :inputs (?l)
    :domain (Lander ?l)
    :outputs (?bq ?hq ?y)
    :certified (and (ComVisible ?bq ?hq ?y ?l) (ComRange ?bq ?l)
                    (BConf ?bq) (HConf ?hq) (Ray ?y))
  )
  (:stream sample-above
    :inputs (?r)
    :domain (Rock ?r)
    :outputs (?bq)
    :certified (and (Above ?bq ?r) (BConf ?bq))
  )
  (:stream sample-base-motion
    :inputs (?bq1 ?bq2)
    :domain (and (BConf ?bq1) (BConf ?bq2))
    :outputs (?bt)
    :certified (BMotion ?bq1 ?bt ?bq2)
  )
  (:stream sample-head-motion
    :inputs (?hq1 ?hq2)
    :domain (and (HConf ?hq1) (HConf ?hq2))
    :outputs (?ht)
    :certified (HMotion ?hq1 ?ht ?hq2)
  )

  ; Could make the robots communicate via line of sight to each other

  ;(:stream test-cfree-ray-conf
  ;  :inputs (?y ?bq)
  ;  :domain (and (Ray ?y) (BConf ?bq))
  ;  :certified (CFreeRayConf ?bq ?r)
  ;)
)