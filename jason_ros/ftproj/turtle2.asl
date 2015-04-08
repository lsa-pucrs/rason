/* Initial beliefs and rules */
turnRate(5).
centerThreshold(7).
/* Initial goals */
!start.

/* Plans */

+!start : true <- 
.wait(10000); 
.print("ZZZzzz...");
.wait(10000);
.print("Finding turtle1 turning 5 degrees, turned 0 so far. Threshold is 7");
.wait(4000);
.print("Finding turtle1 turning 5 degrees, turned 5 so far. Threshold is 7");
.wait(2000);
.print("Found turtle1, but I need to reposition myself...");
.wait(5000);
.print("Found turtle1");
.print("helper ready.");
.print("I'm ready.");
.wait(4000);
.print("reporting to turtle1 -> -5 1.28 -85   -5 1.28 -85");
.wait(1500);
.print("turtle1 said 'thanks'...");
.print("You're welcome, turtle1").

