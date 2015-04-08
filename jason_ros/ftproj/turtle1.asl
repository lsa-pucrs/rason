// Agent robot in project robot.mas2j

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start <- .wait(17000);
.print("Moving to Destination");
.wait(1000);
.print("something went wrong...no_progress");
.print("Running local diagnostics");
.print("diagnose alone has failed");
.print("Finished diagnostics");
.print("Moving to Destination");
.wait(1000);
.print("Cannot solve problem alone, asking for help.");
.wait(14000);
.print("turtle2 found me, asking to track me");
.wait(1000);
.print("Trying to bit for turtle2");
.wait(700);
.print(" requesting some feedback from turtle2");
.wait(500);
.print("turtle2 has reported as initial info( pos -5 degrees, depth 1.28 meters and yaw -85 degrees )");
.print("and current info( pos -5 degrees, depth 1.28 meters and yaw-85 degrees )");
.print("fault in both actuators, thanks turtle2").

+!goToDestination
	<- .print("Moving to Destination");
	   move(1);
	   move(-1);
	   .print("Arriving at destination").

-!goToDestination[R1 | Rest] 
	: triedAlone
	<- .print("Cannot solve problem alone, asking for help.");
	   +reason(R1);
	   .broadcast(achieve,helpme).

-!goToDestination[R1 | Rest]
	<- .print("something went wrong...",R1);
		!diagnoseAlone([R1| Rest]);
		!goToDestination.

+!diagnoseAlone(Reasons) 
	: .member(no_progress,Reasons) 
	| .member(bad_move,Reasons)
//	& .member(error_msg(M),Reasons)
	<- .print("Running local diagnostics");
		.print("diagnose alone has failed");
		+triedAlone;
		.print("Finished diagnostics").
		
//Plans to help diagnose
+!tryMove[source(R)]
	<- .print("Trying to bit for ",R);
	   move(0.3);
	   .print("It was just a false positive.");
	   -reason(_);
	   -triedAlone;
	   .send(R,achieve,thanks).
		  
-!tryMove[source(R)]		
	<- .print(" requesting some feedback from ",R);
	   .send(R,achieve,givemeFeedback).

+found(Me)[source(R)] 
	: .my_name(Me)
	<- .print(R," found me, asking to track me");
	   .send(R,achieve,trackMe).
	   
+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)] 
    : reason(no_progress)
    & ( not (Depth == Depth1) | not (Deg == Deg1) )
	<- .print(Helper," has reported as initial info( pos ",Deg," degrees, depth ",Depth," meters and yaw ",Yaw," degrees )");
       .print("and current info( pos ",Deg1," degrees, depth ",Depth1," meters and yaw",Yaw1," degrees )");
	   .print("fault in odometry system, thanks ", R);
	   .send(R,achieve,thanks).

+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(no_progress)
    & Depth == Depth1
    & Deg   == Deg1
	<- .print(Helper," has reported as initial info( pos ",Deg," degrees, depth ",Depth," meters and yaw ",Yaw," degrees )");
       .print("and current info( pos ",Deg1," degrees, depth ",Depth1," meters and yaw",Yaw1," degrees )");
	   .print("fault in both actuators, thanks ",R);
	   .send(R,achieve,thanks).

+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(bad_move)
    & (Yaw == Yaw1)
	<- .print(Helper," has reported as initial info( pos ",Deg," degrees, depth ",Depth," meters and yaw ",Yaw," degrees )");
       .print("and current info( pos ",Deg1," degrees, depth ",Depth1," meters and yaw",Yaw1," degrees )");
	   .print("it was a false positive for bad_move, thanks ",R);
	   .send(R,achieve,thanks).

+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(bad_move)
    & (Yaw1 > Yaw)
	<- .print(Helper," has reported as initial info( pos ",Deg," degrees, depth ",Depth," meters and yaw ",Yaw," degrees )");
       .print("and current info( pos ",Deg1," degrees, depth ",Depth1," meters and yaw",Yaw1," degrees )");
	   .print("right actuator has problems, thanks ",R);
	   .send(R,achieve,thanks).
	   
+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(bad_move)
    & (Yaw1 < Yaw)
	<- .print(Helper," has reported as initial info( pos ",Deg," degrees, depth ",Depth," meters and yaw ",Yaw," degrees )");
       .print("and current info( pos ",Deg1," degrees, depth ",Depth1," meters and yaw",Yaw1," degrees )");
	   .print("left actuator has problems, thanks ",R);
	   .send(R,achieve,thanks).

