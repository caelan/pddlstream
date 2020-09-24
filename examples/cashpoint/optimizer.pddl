(define (stream cashpoint)
  ;(:function (Dist ?q1 ?q2)
  ;  (and (Conf ?q1) (Conf ?q2))
  ;)

  ;;;;;;;;;;

  (:optimizer gurobi

    ; One var per machine
    ; Create vars for before/after transaction
    ; Condition on the initial cash values?
    (:variables (?wc ?pc2 ?mc2)
      :inputs (?m) % ?pc1 ?mc1)
      :domain (and (machine ?m)) ; (pcash ?pc1) (mcash ?mc1))
      :graph (and (wcash ?wc) (pcash ?pc2) (mcash ?mc2)
                  (transaction ?wc ?pc2 ?mc2)

      ))

    (:constraint (ge ?c1 ?c2)
      :necessary (and (pcash ?c1) (tcash ?c2)))

    (:constraint (sum ?pc1 ?wc ?pc2)
      :necessary (and (pcash ?pc1) (wcash ?wc) (pcash ?pc2)))

    ;(:constraint (sum ?mc2 ?wc ?mc1)
    ;  :necessary (and (mcash ?mc2) (wcash ?wc) (mcash ?mc1)))

    ;(:objective Dist)
  )
)