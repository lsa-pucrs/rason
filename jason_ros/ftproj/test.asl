// Agent test in project robot.mas2j

/* Initial beliefs and rules */
num(1,2,3,-4).

/* Initial goals */

!start.

/* Plans */

+!start : num(A,B,C,D) & math.abs( (math.abs(A) + math.abs(B))-(math.abs(C) + math.abs(D)) ) < 8 <- .print("ok").



