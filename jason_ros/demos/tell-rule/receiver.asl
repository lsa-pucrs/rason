// Agent receiver in project tell-rule.mas2j

/* Initial beliefs */

b.
c.

/* Plans */

+!test : a <- .print(ok).
+!test     <- .print(not_ok).

// customisation of KQML performative tellRule
+!kqml_received(A,tellRule,Rule,_)
   <- .print("Received rule(s) ",Rule, " from ",A);
      rules.add_rule(Rule);
      // get all rules and print them
      rules.get_rules(_,LR);
      .print("Rules: ",LR).
      
