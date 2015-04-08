/* Initial beliefs and rules */
turnRate(5).
centerThreshold(7).
/* Initial goals */
!start.

/* Plans */

+!start : true <- .wait(4000); .print("ZZZzzz...").

+!helpme[source(R)] : true 
	<- +helping(R);
	   !findRobot(R,0);
	   .print("helper ready.");
	   .send(R,tell,found(R)).

+!findRobot(Robot,Turned) : robot(Robot,Deg,Depth,Yaw) & centerThreshold(DT) & math.abs(Deg) >= DT
	<- .print("Found ",Robot, ", but I need to reposition myself...");
	   turn(Deg);
	   !findRobot(Robot,Turned+Deg).
	
+!findRobot(Robot,Turned) : robot(Robot,Deg,Depth,Yaw) & centerThreshold(DT) & math.abs(Deg) < DT
	<- .print("Found ",Robot);
	   +foundRobot(Robot,Deg,Depth,Yaw).
	
+!findRobot(Robot,Turned) : turnRate(Rate) & centerThreshold(DT)
	<- .print("Finding ",Robot," turning ",Rate," degrees, turned ",Turned," so far. Threshold is ",DT);
	   turn(Rate);
	   !findRobot(Robot,Turned+Rate).

+!trackMe[source(R)] : helping(R) & foundRobot(Robot,Deg,Depth,Yaw)
	<-  .send(R,achieve,tryMove).

+!givemeFeedback[source(R)] : helping(R) & foundRobot(Robot,Deg,Depth,Yaw) & robot(Robot1,Deg1,Depth1,Yaw1)
	<- .print(Deg," ",Depth," ",Yaw," ",Deg1," ",Depth1," ",Yaw1); 
	.send(R,achieve,feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))).
	   
+!thanks[source(R)] : helping(R)
	<- .print(R," said 'thanks'...");
	   .print("You're welcome, ",R);
	   -helping(R).
	   
+robot(Robot,Deg,Depth,Yaw) 
	<- -+ robot(Robot,Deg,Depth,Yaw);
	.print(".").// Robot, " detected ", Deg, " degrees ", Depth, " meters and yaw=",Yaw).
	   //-robot(Robot,_,_,_);
	   //+robot(Robot,Deg,Depth,Yaw).

		
		
