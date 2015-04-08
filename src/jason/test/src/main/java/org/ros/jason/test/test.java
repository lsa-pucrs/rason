package org.ros.jason.test;

import org.ros.concurrent.CancellableLoop;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.ros.internal.message.Message;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.w3c.dom.DOMException;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class test extends AbstractNodeMain {
	
	ConnectedNode connectedNode;
	Map< String, Publisher<Message> > publisherMap;
	Map< String, NodeList > actionMap;
	
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("jason/test");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
	  
	this.connectedNode = connectedNode;
	
	publisherMap = new HashMap< String, Publisher<Message> >();
	actionMap    = new HashMap< String, NodeList >();
	
	Load("ROS-JasonActions.xml");

    // This CancellableLoop will be canceled automatically when the node shuts
    // down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {

      @Override
      protected void setup() {
      }
      
      @Override
      protected void loop() throws InterruptedException {
    	  
    		String [] vars = new String[1];
    		vars[0] = "17.4";
    		
    		if ( !Publish("move_forward", vars ) ) {
    			
    			System.out.println( "===> action failed." );
    		}
    		
    	  
    	  
        Thread.sleep(1000);
      }
    });
  }
  public boolean Publish( String action, Object [] vars ) {

	  boolean ret = false;

	  if ( actionMap.containsKey(action) ) {
		  
		  NodeList topics = actionMap.get(action);
		  final int topicsLen = topics.getLength();
		  for( int index = 0; index < topicsLen; index++ ) {
			  
			  Node currTopic = topics.item(index);
			  if ( !currTopic.getNodeName().equals("topic") ) {
				  
				  continue;
			  }
			  

			  Message newMessage = CreateMessage(
					  currTopic.getAttributes().getNamedItem("type").getNodeValue(), 
					  currTopic.getChildNodes(), 
					  vars );
			  
			  Publisher<Message> p = publisherMap.get( currTopic.getAttributes().getNamedItem("name").getNodeValue() );
			  p.publish( newMessage );
		  }
		  ret = true;
	  }
	  else {
		  
		  System.out.println("===> action " + action + " not found!");
	  }
	  return ret;
  }
  
  public boolean Load( String filename ) {
	  
	  try {
		  
		  File file = new File( filename );
		  DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
		  DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
		  Document doc = dBuilder.parse(file);
		  
		  NodeList  actions   = doc.getElementsByTagName("action");
		  final int actionsLen = actions.getLength();
		  for( int actionsIndex = 0; actionsIndex < actionsLen; actionsIndex++ ) {
			  
			  Node currAction = actions.item( actionsIndex );
			  
			  actionMap.put(
					  currAction.getAttributes().getNamedItem("name").getNodeValue(),
					  currAction.getChildNodes() );
			  
			  NodeList topics = currAction.getChildNodes();
			  final int topicsLen = topics.getLength();
			  for( int topicsIndex = 0; topicsIndex < topicsLen; topicsIndex++ ) {
				  
				  Node currTopic = topics.item( topicsIndex );
				  if ( !currTopic.getNodeName().equals("topic") ) {
					  
					  continue;
				  }
			  
				  Publisher<Message> newPublisher = connectedNode.newPublisher(
						  currTopic.getAttributes().getNamedItem("name").getNodeValue()	,
						  currTopic.getAttributes().getNamedItem("type").getNodeValue() );
				  publisherMap.put( currTopic.getAttributes().getNamedItem("name").getNodeValue(), newPublisher );
			  }
		  }
	  }
	  catch ( Exception e ) {
	  
		  e.printStackTrace();
		  return false;
	  }
	  return true;
  }
  
  public Message CreateMessage( String messageType, NodeList rosVars, Object [] vars ) {
	  
	  Message ret = connectedNode.getTopicMessageFactory().newFromType( messageType );
	  
	  for( int index = 0; index < rosVars.getLength(); index++ ) {
		  
		  Node currNode = rosVars.item( index );
		  if ( !currNode.getNodeName().equals( "item" ) ) {
			  
			  continue;
		  }
		  
		  String varName  = currNode.getAttributes().getNamedItem( "name"  ).getNodeValue();
		  String varType  = currNode.getAttributes().getNamedItem( "type"  ).getNodeValue();
		  String varValue;
		  
		  try {
			  
			  varValue = currNode.getAttributes().getNamedItem( "value" ).getNodeValue();
			  if ( varValue.length() > 5 && varValue.substring(0, 5).equalsIgnoreCase("$parm") ) {

				  int parmIndex;
				  try  
				  {  
					  parmIndex = Integer.parseInt( varValue.substring(5) );  
					  varValue = (String)vars[ parmIndex ];
				  }  
				  catch(NumberFormatException nfe)  
				  { 
					  parmIndex = -1;
				  }  
			  }
		  }
		  catch( DOMException exception ) {
			
			  varValue = "";
		  }
		  catch( NullPointerException e ) {
			  
			  varValue = "";
		  }

		  System.out.println("var " + varName + "  value = " + varValue );
		  
    	  if ( varType.equalsIgnoreCase("bool") ) {

    		  ret.toRawMessage().setBool( varName, Boolean.parseBoolean( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("int8") ) {
    		  
    		  ret.toRawMessage().setInt8( varName, Byte.parseByte( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("uint8") ) {

    		  ret.toRawMessage().setUInt8( varName, Byte.parseByte( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("int16") ) {
    		  
    		  ret.toRawMessage().setInt16( varName, Short.parseShort( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("uint16") ) {

    		  ret.toRawMessage().setUInt16( varName, Short.parseShort( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("int32") ) {
    		  
    		  ret.toRawMessage().setInt32( varName, Integer.parseInt( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("uint32") ) {
    		  
    		  ret.toRawMessage().setUInt32( varName, Integer.parseInt( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("int64") ) {
    		  
    		  ret.toRawMessage().setInt64( varName, Long.parseLong( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("uint64") ) {
	  		
    		  ret.toRawMessage().setUInt64( varName, Long.parseLong( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("float32") ) {
    		  
    		  ret.toRawMessage().setFloat32( varName, Float.parseFloat( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("float64") ) {
    		  
    		  ret.toRawMessage().setFloat64( varName, Double.parseDouble( varValue ) );
    	  }
    	  else if ( varType.equalsIgnoreCase("string") ) {
    		  
    		  ret.toRawMessage().setString( varName, varValue );
    	  }
    	  else {
    		  
    		  ret.toRawMessage().setMessage( varName, CreateMessage( varType, currNode.getChildNodes(), vars ) );
    	  }
	  }
	return ret;
  }
}
