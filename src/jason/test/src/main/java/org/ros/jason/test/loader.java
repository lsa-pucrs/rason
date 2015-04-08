package org.ros.jason.test;

import org.ros.jason.test.test;

class loader {
	
	public static void main(String[] args) {
		
		test node = new test();
		
		for( int index = 0; index < args.length; index++ ) {
			
			node.Load( args[ index ] );
		}
	}
}