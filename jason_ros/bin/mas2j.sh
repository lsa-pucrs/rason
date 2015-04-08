#!/bin/sh
#
# this script creates the Ant script (build.xml) 
# to run a Jason project  
#

if [ ! -f $1 ]
then
    echo File $1 not found
    exit
fi


CURDIR=`pwd`

JASONDIR=`dirname $0`/..
cd $JASONDIR
JASONDIR=`pwd`

cd $CURDIR

java -classpath bin/classes:$JASONDIR/lib/jason.jar:$JASONDIR/lib/guava-12.0.jar:$JASONDIR/lib/rosjason-0.1.0.jar:$JASONDIR/lib/jade.jar:$JASONDIR/lib/c4jason.jar:$JASONDIR/lib/cartago.jar:$JASONDIR/lib/jacamo.jar:$CLASSPATH:. \
  jason.mas2j.parser.mas2j $1 $2

#chmod u+x *.sh
