// Agent z in project robot.mas2j

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start : true <- .print("hello world."); move(1).

+!checkposition(X,Y) <- move(X,Y).
-!checkposition(X,Y) <- .print( "Position ",X,",",Y, " unreachable" ).

+warning(X,Y) <- .print("warning received from coordinates ",X,",",Y,", checking..."); !checkposition(X,Y).

