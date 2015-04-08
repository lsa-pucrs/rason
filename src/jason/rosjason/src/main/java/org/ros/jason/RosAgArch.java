package org.ros.jason;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import jason.architecture.AgArch;
import jason.asSyntax.Literal;
import jason.asSemantics.ActionExec;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import org.ros.exception.RosRuntimeException;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

public class RosAgArch extends AgArch {
	
	RosNode rosNode;
	Map<Integer,ActionExec> rosPendingActions;
	
    @Override
    public void init() throws Exception {
    	
        super.init();
        
        rosPendingActions = new HashMap< Integer,ActionExec >();
        
        System.out.println( getAgName() + " : Connecting to ROS Master..." );
        String[] nodeClassName = { "org.ros.jason.RosNode" };
        CommandLineLoader loader = new CommandLineLoader(Lists.newArrayList(nodeClassName));
        NodeConfiguration nodeConfiguration = loader.build();
        
        nodeConfiguration.setNodeName( "/jason/" + getAgName() );
        
        NodeMain nodeMain = null;
        try {
        	
          nodeMain = loader.loadClass(nodeClassName[0]);
        } 
        catch (ClassNotFoundException e) {
        	
          throw new RosRuntimeException("Unable to locate node: " + nodeClassName[0], e);
        } 
        catch (InstantiationException e) {
        	
          throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName[0], e);
        } 
        catch (IllegalAccessException e) {
        	
          throw new RosRuntimeException("Unable to instantiate node: " + nodeClassName[0], e);
        }

        Preconditions.checkState(nodeMain != null);
        NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
        nodeMainExecutor.execute(nodeMain, nodeConfiguration);        
            
        rosNode = (RosNode)nodeMain;
        
        while( !rosNode.Connected() ) Thread.sleep(1000);
        System.out.println( "Ready." );
    }
    
    @Override
    public void reasoningCycleStarting() {
    	
    	if ( rosNode.hasEvents() ) {
    		
    		Map< String, List< String > >  events = rosNode.getEvents();
    		
    		for ( Map.Entry< String, List< String > > entry : events.entrySet() ) {
    			Integer actionId = Integer.parseInt( entry.getValue().get(0) );
    			ActionExec action = rosPendingActions.get( actionId );
    			if ( entry.getValue().get(1).equalsIgnoreCase( "OK" )  ) {
    			
    				action.setResult( true );
    			}
    			else {
    				
    				action.setResult( false );
    				action.setFailureReason( Literal.parseLiteral( entry.getValue().get(1) ), 
    						entry.getValue().size() > 2 ? entry.getValue().get(2) : "" );
    			}
        		getTS().getC().addFeedbackAction( action );
    	        rosPendingActions.remove( actionId );
    		}
	        rosNode.ClearEvents();
	        wake();
    	}
    	super.reasoningCycleStarting();
    }
    
    @Override
    public List<Literal> perceive() {
    	
    	List<Literal> ret = new ArrayList< Literal >();
    	if ( rosNode.hasPerceptions() ) {
    		
    		Map< String, List< String > >  events = rosNode.getPerceptions();
    		for ( Map.Entry< String, List< String > > entry : events.entrySet() ) {
    			
    			String source = entry.getKey();
    			List< String > perceptionList = entry.getValue();
    			
		    	for ( int nIndex = 0; nIndex < perceptionList.size(); nIndex++ ) {
		    		
		    		System.out.println("new perception = " + perceptionList.get( nIndex ) );
		    		ret.add( Literal.parseLiteral( perceptionList.get( nIndex ) ) );
		    	}
    		}
	    	rosNode.ClearPerceptions();
    	}
    	return ret;
    }
    
    @Override
    public void act(ActionExec act, List<ActionExec> fb) {

    	List<String> parms = new ArrayList<String>();
    	
    	for( int index = 0; index < act.getActionTerm().getArity(); index++  ) {
    		
    		parms.add( act.getActionTerm().getTerm( index ).toString() );
    	}
		Integer rosActionId = rosNode.SendAction( act.getActionTerm().getFunctor(), parms );
		rosPendingActions.put( rosActionId, act );
    }
}
