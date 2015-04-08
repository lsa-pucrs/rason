package jason.bb;

import jason.asSemantics.Agent;
import jason.asSemantics.Unifier;
import jason.asSyntax.Literal;
import jason.asSyntax.PredicateIndicator;
import jason.bb.BeliefBase;
import jason.bb.DefaultBeliefBase;

import java.util.Iterator;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 
 This class is to be extended by customised belief bases that may be 
 used in chains (of responsibility).   
 
 For example, the code
 <pre>
 BeliefBase bb = 
    new MyBB1(
       new MyBB2());
      // DefaultBeliefBase is the next of the last element of the chain
 </pre>
 will chain 3 BBs: MyBB1, myBB2, and the DefaultBeliefBase. So, for any operation of 
 the BB interface, the operation is firstly called in MyBB1, then in MyBB2 and finally
 in the DefaultBeliefBase. 
 
 The code of MyBB1 looks like:
 
 <pre>
 class MyBB1 extends ChainBBAdapter {
    public MyBB1() { }
    public MyBB1(BeliefBase next) {
        super(next);
    }
 
    public boolean add(Literal l) {
        ... some customisation of add ....
        return next.add(l); // delegate the operation for the next BB in the chain
    }
    
    ... customisation of other operations ...
 }
 </pre>
 
 @navassoc - nextBB - BeliefBase
 
 @author Jomi
 
 */
@SuppressWarnings("deprecation")
public abstract class ChainBBAdapter implements BeliefBase {

    protected BeliefBase nextBB = null; // the next BB in the chain
    
    public ChainBBAdapter() {
        nextBB = new DefaultBeliefBase();        
    }
    public ChainBBAdapter(BeliefBase bb) {
        nextBB = bb;
    }

    public void setNext(BeliefBase bb) {
        nextBB = bb;
    }
    
    public ChainBBAdapter getNextAdapter() {
        return nextBB instanceof ChainBBAdapter ? (ChainBBAdapter)nextBB : null;
    }
    
    public BeliefBase getLastBB() {
        if (nextBB == null)
            return this;
        else if (nextBB instanceof ChainBBAdapter) 
            return ((ChainBBAdapter)nextBB).getLastBB();
        else 
            return nextBB;            
    }

    // Methods of BB interface
    
    public void init(Agent ag, String[] args) {
        nextBB.init(ag, args);
    }
    public void stop() {
        nextBB.stop();
    }

    public void clear() {
        nextBB.clear();
    }
    
    public boolean add(Literal l) {
        return nextBB.add(l);
    }

    public boolean add(int index, Literal l) {
        return nextBB.add(index, l);
    }

    public Literal contains(Literal l) {
        return nextBB.contains(l);
    }

    public Iterator<Literal> getAll() {
        return nextBB.getAll();
    }

    public Iterator<Literal> iterator() {
        return nextBB.iterator();
    }

    public Iterator<Literal> getCandidateBeliefs(PredicateIndicator pi) {
        return nextBB.getCandidateBeliefs(pi);
    }
    
    public Iterator<Literal> getCandidateBeliefs(Literal l, Unifier u) {
        return nextBB.getCandidateBeliefs(l, u);
    }

    public Iterator<Literal> getPercepts() {
        return nextBB.getPercepts();
    }

    public Iterator<Literal> getRelevant(Literal l) {
        return nextBB.getRelevant(l);
    }

    public boolean abolish(PredicateIndicator pi) {
        return nextBB.abolish(pi);
    }

    public boolean remove(Literal l) {
        return nextBB.remove(l);
    }

    public int size() {
        return nextBB.size();
    }

    public Element getAsDOM(Document document) {
        return nextBB.getAsDOM(document);
    }

    @Override
    public BeliefBase clone() {
        return this;
    }
    
    @Override
    public String toString() {
        return nextBB.toString();
    }
}
