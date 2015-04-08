package org.ros.jason;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.concurrent.ConcurrentHashMap;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.*;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class RosNode extends AbstractNodeMain {
	
	ConnectedNode connectedNode;
	boolean	connected;
	int	action_id = -1;
	
	Publisher<jason_msgs.action> actionPub;
	
	Subscriber<jason_msgs.perception> perceptionSub;
	Map<String, List< String > > perceptions;

	Subscriber<jason_msgs.event> eventSub;
	Map<String, List< String > > events;

	@Override
	public GraphName getDefaultNodeName() {

		return GraphName.of("jason/agent");
	}
	
	public boolean hasEvents() {
		
		return !events.isEmpty();
	}
	
	public Map< String, List< String > > getEvents() {
		
		return events;
	}
	
	public void ClearEvents() {
		
		events.clear();
	}
	
	public void ClearPerceptions() {
		
		perceptions.clear();
	}

	public boolean hasPerceptions() {
		
		return !perceptions.isEmpty();
	}
	
	public Map< String, List< String > > getPerceptions() {
		
		return perceptions;
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		
		actionPub = connectedNode.newPublisher( "/jason/action", jason_msgs.action._TYPE );
		
		eventSub = connectedNode.newSubscriber( "/jason/event", jason_msgs.event._TYPE );
		events = new ConcurrentHashMap< String, List< String > >();
		
		eventSub.addMessageListener(new MessageListener<jason_msgs.event>() {
			
		      @Override
		      public void onNewMessage(jason_msgs.event message) {

	    		  System.out.println("agent "+ message.getAgent() +"  event " + message.getParameters().get(0) + " received.");
		    	  if ( message.getAgent().equals( connectedNode.getName().toString() ) ) {
		    		  
		    		  events.put(message.getParameters().get(0), message.getParameters() );
		    	  }
		      }
		    });
		
		perceptionSub = connectedNode.newSubscriber( "/jason/perception", jason_msgs.perception._TYPE );
		perceptions = new ConcurrentHashMap< String, List< String > >();

		perceptionSub.addMessageListener(new MessageListener<jason_msgs.perception>() {
			
		      @Override
		      public void onNewMessage(jason_msgs.perception message) {
		    	  
				  System.out.println("rosjava : received perception " + message.getPerception() + " from " + message.getSource() );
				  perceptions.put( message.getSource(), message.getPerception() );
		      }
		    });


		this.connectedNode = connectedNode;
		this.connected     = true;
	}
	
	public boolean Connected() {
		
		return this.connected;
	}
	
	public int SendAction( String action, List<String> parms ) {
		
		jason_msgs.action act = connectedNode.getTopicMessageFactory().newFromType( jason_msgs.action._TYPE );
		act.setAgent( connectedNode.getName().toString() ); 
		act.setAction( action );
		act.setParameters( parms );
		act.setActionId( ++action_id );
		actionPub.publish( act );
	
		return action_id;
	}
}
