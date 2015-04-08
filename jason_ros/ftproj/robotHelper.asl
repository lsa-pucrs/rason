/* Initial beliefs and rules */
turnRate(2).
centerThreshold(5).
/* Initial goals */

!findRobot("Tbot2",0).

/* Plans */

+!findRobot(Robot,Turned) : robot(Robot,Dir,Deg,Depth) & centerThreshold(DT) & Deg < DT
	<- .puts("Found #{Robot}").
	
+!findRobot(Robot,Turned) : turnRate(Rate) & centerThreshold(DT)
	<- .puts("Finding #{Robot}, turning #{Rate} degrees, turned #{Turned} so far. Threshold is #{DT}");
		turn(right,Rate);
//		.wait(500);
	   !findRobot(Robot,Turned+Rate).

+robot(Robot,Dir,Deg,Depth) : centerThreshold(DT) & Deg < DT
	<-  +robot(Robot,Dir,Deg,Depth);
		.puts("#{Robot} detected #{Deg} degrees #{Dir}").

