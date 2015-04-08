package jason.asSemantics;

import jason.asSyntax.Trigger;

/** call-back interface to be notified about events on goals */
public interface GoalListener {
    
    public enum GoalStates { started, suspended, resumed, finished, failed } ; 
    
    /** method called when a new goal is produced by operator ! */
    public void goalStarted(Event goal); 
    
    /** method called when a goal is successfully finished */
    public void goalFinished(Trigger goal);

    /** method called when a goal is failed */
    public void goalFailed(Trigger goal);
    
    /** method called when a goal is suspended (waiting action on the environment or due to internal actions like .wait and .suspend) */
    public void goalSuspended(Trigger goal, String reason);
    
    /** called when a suspended goal is resumed */
    public void goalResumed(Trigger goal);
    
}
