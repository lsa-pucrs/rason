// Agent robot in project robot.mas2j

/* Initial beliefs and rules */

/* Initial goals */

!goToDestination.

/* Plans */

+!goToDestination : true
	<- .puts("Moving to Destination");
		move(forward,1);move(backward,1);
		.puts("Arriving at destination").

-!goToDestination[R1 | Rest] : triedAlone
	<- .puts("Cannot solve problem alone, communicating with robotHelper");
		.broadcast(achieve,helpme).
	
-!goToDestination[R1 | Rest] : true
	<- .puts("#{R1} #{Rest}");
		!diagnoseAlone([R1| Rest]);
		!goToDestination.

+!diagnoseAlone(Reasons) : .member(blocked,Reasons) & .member(error_msg(M),Reasons)
	<- .puts("Running local diagnostics");
		.puts("diagnose alone has failed");
		+triedAlone;
		.puts("Agent is blocked and moved #{M}");
		.puts("Finished diagnostics").

