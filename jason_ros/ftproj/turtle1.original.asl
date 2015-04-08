// Agent robot in project robot.mas2j

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start <- .wait(6000); !goToDestination.

+!goToDestination : true
	<- .print("Moving to Destination");
	   move(1);
	   move(-1);
	   .print("Arriving at destination").

-!goToDestination[R1 | Rest] : triedAlone
	<- .print("Cannot solve problem alone, asking for help.");
	   +reason(R1);
	   .broadcast(achieve,helpme).

-!goToDestination[R1 | Rest] : true
	<- .print(R1," ",Rest);
		!diagnoseAlone([R1| Rest]);
		!goToDestination.

+!diagnoseAlone(Reasons) : .member(no_progress,Reasons) & .member(error_msg(M),Reasons)
	<- .print("Running local diagnostics");
		.print("diagnose alone has failed");
		+triedAlone;
		.print("Agent is blocked and moved ",M);
		.print("Finished diagnostics").
		
//Plans to help diagnose
+!tryMove[source(R)] : true
	<-  .print("Trying to bit for ",R);
		move(0.3).
		  
-!tryMove[source(R)] : true		
	<- .print(" requesting some feedback from ",R);
	   .send(R,achieve,givemeFeedback).

+found(Me)[source(R)] : .my_name(Me)
	<- .print(R," found me, asking to track me");
	   .send(R,achieve,trackMe).
	   
+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)] 
    : reason(no_progress)
    & ( not (Depth == Depth1) | not (Deg == Deg1) )
	<- .print("fault in odometry system, thanks ", R);
	   .send(R,achieve,thanks).

+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(no_progress)
    & Depth == Depth1
    & Deg   == Deg1
	<- .print("fault in both actuators");
	   .send(R,achieve,thanks).

+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
    : reason(bad_move)
    & ( math.abs( Deg + Yaw ) == math.abs ( Deg1 + Yaw1 ) )
	<- .print("false positive for bad_move");
	   .send(R,achieve,thanks).

//+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
//    : reason(bad_move)
//    & ( ( Deg + Yaw ) > ( Deg1 + Yaw1 ) )
//	<- .print("fault in odometry system");
//	   .send(R,achieve,thanks).

//+!feedback(initPos(Deg,Depth,Yaw),currPos(Deg1,Depth1,Yaw1))[source(R)]
//    : reason(bad_move)
//    & ( ( Deg + Yaw ) < ( Deg1 + Yaw1 ) )
//	<- .print("fault in odometry system");
//	   .send(R,achieve,thanks).
	
