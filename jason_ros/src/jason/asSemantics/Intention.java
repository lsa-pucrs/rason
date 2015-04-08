//----------------------------------------------------------------------------
// Copyright (C) 2003  Rafael H. Bordini, Jomi F. Hubner, et al.
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
// 
// To contact the authors:
// http://www.inf.ufrgs.br/~bordini
// http://www.das.ufsc.br/~jomi
//
//----------------------------------------------------------------------------

package jason.asSemantics;

import jason.asSyntax.ListTerm;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Structure;
import jason.asSyntax.Term;
import jason.asSyntax.Trigger;

import java.io.Serializable;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.atomic.AtomicInteger;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Represents and Intention (a stack of IntendedMeans).
 * 
 * The comparable sorts the intentions based on the atomic property: 
 * atomic intentions comes first.
 * 
 * @author Jomi & Rafael
 */
public class Intention implements Serializable, Comparable<Intention>, Iterable<IntendedMeans> {

    private static final long serialVersionUID = 1L;
    public  static final Intention EmptyInt = null;
    private static AtomicInteger idCount = new AtomicInteger(0);

    private int     id;
    private int     atomicCount    = 0; // how many atomic intended means there are in the intention
    private boolean isSuspended = false;
    
    private Deque<IntendedMeans> intendedMeans = new ArrayDeque<IntendedMeans>();
    
    //private Trigger initialTrigger = null; // just for adicional information/debug (not really necessary)
    
    //static private Logger logger = Logger.getLogger(Intention.class.getName());

    public Intention() {
        id = idCount.incrementAndGet();
    }

    public int getId() {
        return id;
    }

    public void push(IntendedMeans im) {
        intendedMeans.push(im);
        if (im.isAtomic()) 
            atomicCount++;
        //if (initialTrigger == null)
        //    initialTrigger = im.getTrigger();
    }
    
    public IntendedMeans peek() {
        return intendedMeans.peek();
    }

    public IntendedMeans pop() {
        IntendedMeans top = intendedMeans.pop();

        if (isAtomic() && top.isAtomic()) {
            atomicCount--;
            /* for (IntendedMeans im : intendedMeans) {
                if (im.isAtomic()) {
                    isAtomic = true;
                    break;
                }
            }*/
        }
        return top;
    }

    public boolean isAtomic() {
        return atomicCount > 0;
    }
    
    public void setAtomic(int a) { // used for testing
        atomicCount = a;
    }
    

    public Iterator<IntendedMeans> iterator() {
        return intendedMeans.iterator(); 
    }

    public boolean isFinished() {
        return intendedMeans.isEmpty();
    }

    public int size() {
        return intendedMeans.size();
    }

    public void setSuspended(boolean b) {
        isSuspended = b;
    }
    
    public boolean isSuspended() {
        return isSuspended;
    }
    
    /** returns the IntendedMeans with TE = g, returns null if there isn't one */
    public IntendedMeans getIM(Trigger g, Unifier u) {
        for (IntendedMeans im : intendedMeans)
            //System.out.println(g + " = "+ im.getTrigger()+" = "+u.unifies(g, im.getTrigger()));
            if (u.unifies(g, im.getTrigger()))
                return im;
        return null;
    }
    
    public IntendedMeans getBottom() {
        return intendedMeans.getLast();
    }
    
    /** returns true if the intention has an IM where TE = g, using u to verify equality */
    public boolean hasTrigger(Trigger g, Unifier u) {
        //return getIM(g,u) != null;
        for (IntendedMeans im : intendedMeans)
            if (u.unifies(g, im.getTrigger()))
                return true;
        return false;
    }

    /** remove all IMs until the IM with trigger te */
    public boolean dropGoal(Trigger te, Unifier un) {
        IntendedMeans im = getIM(te, un);
        if (im != null) {
            // remove the IMs until im-1
            while (peek() != im) {
                pop();
            }
            pop(); // remove im
            return true;
        }      
        return false;
    }
    
    /** implements atomic intentions > not atomic intentions */
    public int compareTo(Intention o) {
        if (o.atomicCount > this.atomicCount) return 1;
        if (this.atomicCount > o.atomicCount) return -1;
        return 0;
    }
    
    public boolean equals(Object o) {
        if (o == null) return false;
        if (o == this) return true;
        if (o instanceof Intention) return ((Intention)o).id == this.id;
        return false;
    }
    
    public int hashCode() {
        return String.valueOf(id).hashCode();
    }
    
    public Intention clone() {
        Intention i = new Intention();
        i.id = id;
        i.atomicCount = atomicCount;
        i.intendedMeans = new ArrayDeque<IntendedMeans>();
        for (IntendedMeans im: intendedMeans) {
            i.intendedMeans.add((IntendedMeans)im.clone());
        }
        return i;
    }
        
    public String toString() {
        StringBuilder s = new StringBuilder("intention "+id+": \n");
        for (IntendedMeans im: intendedMeans)
            s.append("    " + im + "\n");
        if (isFinished())
            s.append("<finished intention>");
        return s.toString();
    }

    public Term getAsTerm() {
        Structure intention = new Structure("intention");
        intention.addTerm(new NumberTermImpl(getId()));
        ListTerm lt = new ListTermImpl();
        for (IntendedMeans im: intendedMeans)
            lt.add(im.getAsTerm());
        intention.addTerm(lt);
        return intention;        
    }

    /** get as XML */
    public Element getAsDOM(Document document) {
        Element eint = (Element) document.createElement("intention");
        eint.setAttribute("id", id + "");
        for (IntendedMeans im: intendedMeans)
            eint.appendChild(im.getAsDOM(document));
        //if (intendedMeans.isEmpty())
        //    eint.appendChild( initialTrigger.getAsDOM(document));        
        eint.setAttribute("finished", ""+isFinished());

        return eint;
    }

}
