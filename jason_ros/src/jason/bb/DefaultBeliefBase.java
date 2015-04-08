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

package jason.bb;

import jason.asSemantics.Agent;
import jason.asSemantics.Unifier;
import jason.asSyntax.Literal;
import jason.asSyntax.PredicateIndicator;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

/**
 * Default implementation of Jason BB.
 */
public class DefaultBeliefBase implements BeliefBase {

    private static Logger logger = Logger.getLogger(DefaultBeliefBase.class.getSimpleName());

    /**
     * belsMap is a table where the key is the bel.getFunctorArity and the value
     * is a list of literals with the same functorArity.
     */
    private Map<PredicateIndicator, BelEntry> belsMap = new ConcurrentHashMap<PredicateIndicator, BelEntry>();

    private int size = 0;

    /** set of beliefs with percept annot, used to improve performance of buf */
    protected Set<Literal> percepts = new HashSet<Literal>();

    public void init(Agent ag, String[] args) {
        if (ag != null) {
            logger = Logger.getLogger(ag.getTS().getUserAgArch().getAgName() + "-"+DefaultBeliefBase.class.getSimpleName());
        }
    }

    public void stop() {
    }

    public int size() {
        return size;
    }
    
    public void clear() {
        size = 0;
        percepts.clear();
        belsMap.clear();
    }

    public Iterator<Literal> getPercepts() {
        final Iterator<Literal> i = percepts.iterator();
        return new Iterator<Literal>() {
            Literal current = null;
            public boolean hasNext() {
                return i.hasNext();
            }
            public Literal next() {
                current = i.next(); 
                return current;
            }
            public void remove() {
                if (current == null) {
                    logger.warning("Trying to remove a perception, but the the next() from the iterator is not called before!");
                }
                // remove from percepts
                i.remove();
                
                // remove the percept annot
                current.delAnnot(BeliefBase.TPercept);
                
                // and also remove from the BB
                removeFromEntry(current);
            }
        };
        //return ((Set<Literal>)percepts.clone()).iterator();        
    }

    Set<Literal> getPerceptsSet() {
        return percepts;
    }
    
    public boolean add(Literal l) {
        return add(l, false);
    }
    
    public boolean add(int index, Literal l) {
        return add(l, index != 0);
    }
    
    protected boolean add(Literal l, boolean addInEnd) {
        if (!l.canBeAddedInBB()) {
            logger.log(Level.SEVERE, "Error: '"+l+"' can not be added in the belief base.");
            return false;
        }
        
        Literal bl = contains(l);
        if (bl != null && !bl.isRule()) {
            // add only annots
            if (bl.importAnnots(l)) {
                // check if it needs to be added in the percepts list
                // (note that l contains only the annots imported)
                if (l.hasAnnot(TPercept)) {
                    percepts.add(bl);
                }
                return true;
            }
        } else {
            BelEntry entry = belsMap.get(l.getPredicateIndicator());
            if (entry == null) {
                entry = new BelEntry();
                belsMap.put(l.getPredicateIndicator(), entry);
            }
            l = l.copy(); // we need to clone l for the consequent event to not have a ref to this bel (which may change before the event is processed); see bug from Viviana Marcardi
            entry.add(l, addInEnd);  
            
            // add it in the percepts list
            if (l.hasAnnot(TPercept)) {
                percepts.add(l);
            }
            size++;
            return true;
        }
        return false;
    }
    
    public boolean remove(Literal l) {
        Literal bl = contains(l);
        if (bl != null) {
            if (l.hasSubsetAnnot(bl)) { // e.g. removing b[a] or b[a,d] from BB b[a,b,c]
                                        // second case fails
                if (l.hasAnnot(TPercept)) {
                    percepts.remove(bl);
                }
                boolean result = bl.delAnnots(l.getAnnots()); // note that l annots can be empty, in this case, nothing is deleted!
                return removeFromEntry(bl) || result;
            }
        } else {
            if (logger.isLoggable(Level.FINE)) logger.fine("Does not contain " + l + " in " + belsMap);
        }
        return false;
    }

    private boolean removeFromEntry(Literal l) {
        if (l.hasSource()) {
            return false;
        } else {
            PredicateIndicator key = l.getPredicateIndicator();
            BelEntry entry = belsMap.get(key);
            entry.remove(l);
            if (entry.isEmpty()) {
                belsMap.remove(key);
            }
            size--;
            return true;
        }
    }

    public Iterator<Literal> iterator() {
        final Iterator<BelEntry> ibe = belsMap.values().iterator();
        if (ibe.hasNext()) {
            return new Iterator<Literal>() {

                Iterator<Literal> il = ibe.next().list.iterator();
                Literal l;
                
                public boolean hasNext() {
                    return il.hasNext();
                }

                public Literal next() {
                    l = il.next();
                    if (!il.hasNext() && ibe.hasNext()) {
                        il = ibe.next().list.iterator();
                    }
                    return l;
                }

                public void remove() {
                    il.remove();
                    if (l.hasAnnot(TPercept)) {
                        percepts.remove(l);
                    }
                    size--;
                }
            };
        } else {
            return new ArrayList<Literal>(0).iterator();
        }
    }
    
    /** @deprecated use iterator() instead of getAll */
    public Iterator<Literal> getAll() {
        return iterator();
    }    

    public boolean abolish(PredicateIndicator pi) {
        BelEntry entry = belsMap.remove(pi);
        if (entry != null) {
            size -= entry.size();
            
            // remove also in percepts list!
            Iterator<Literal> i = percepts.iterator();
            while (i.hasNext()) {
                Literal l = i.next();
                if (l.getPredicateIndicator().equals(pi))
                    i.remove();
            }
            return true;
        } else {
            return false;
        }
        //return belsMap.remove(pi) != null;
    }

    public Literal contains(Literal l) {
        BelEntry entry = belsMap.get(l.getPredicateIndicator());
        if (entry == null) {
            return null;
        } else {
            //logger.info("*"+l+":"+l.hashCode()+" = "+entry.contains(l)+" in "+this);//+" entry="+entry);
            return entry.contains(l);
        }
    }

    public Iterator<Literal> getCandidateBeliefs(PredicateIndicator pi) {
        BelEntry entry = belsMap.get(pi);
        if (entry != null)
            return Collections.unmodifiableList(entry.list).iterator();
        else
            return null;
    }
    
    public Iterator<Literal> getCandidateBeliefs(Literal l, Unifier u) {
        if (l.isVar()) {
            // all bels are relevant
            return iterator();
        } else {
            BelEntry entry = belsMap.get(l.getPredicateIndicator());
            if (entry != null)
                return Collections.unmodifiableList(entry.list).iterator();
            else
                return null;
        }
    }
    
    /** @deprecated use getCandidateBeliefs(l,null) instead */ 
    public Iterator<Literal> getRelevant(Literal l) {
        return getCandidateBeliefs(l, null);
    }

    public String toString() {
        return belsMap.toString();
    }

    public BeliefBase clone() {
        DefaultBeliefBase bb = new DefaultBeliefBase();
        for (Literal b: this) {
            bb.add(1, b.copy());
        }
        return bb;
    }
    
    public Element getAsDOM(Document document) {
        int tries = 0;
        Element ebels = null;
        while (tries < 10) { // max 10 tries
            ebels = (Element) document.createElement("beliefs");
            try { 
                for (Literal l: this) 
                    ebels.appendChild(l.getAsDOM(document));
                break; // quit the loop
            } catch (Exception e) { // normally concurrent modification, but others happen
                tries++;
                // simply tries again
            }
        }
        return ebels;
    }
    
    /** each predicate indicator has one BelEntry assigned to it */
    final class BelEntry {
        
        final private List<Literal> list = new LinkedList<Literal>(); // maintains the order of the beliefs
        final private Map<LiteralWrapper,Literal> map = new HashMap<LiteralWrapper,Literal>(); // to find content faster
        
        public void add(Literal l, boolean addInEnd) {
            map.put(new LiteralWrapper(l), l);
            if (addInEnd) {
                list.add(l);
            } else {
                list.add(0,l);
            }
        }
        
        public void remove(Literal l) {
            Literal linmap = map.remove(new LiteralWrapper(l)); 
            if (linmap != null) {
                list.remove(linmap);
            }
        }
        
        public int size() {
            return map.size();
        }
        
        public boolean isEmpty() {
            return list.isEmpty();
        }
        
        public Literal contains(Literal l) {
            return map.get(new LiteralWrapper(l));
        }
        
        protected Object clone() {
            BelEntry be = new BelEntry();
            for (Literal l: list) {
                be.add(l.copy(), false);
            }
            return be;
        }
        
        public String toString() {
            StringBuilder s = new StringBuilder();
            for (Literal l: list) {
                s.append(l+":"+l.hashCode()+",");
            }
            return s.toString();
        }
        
        /** a literal that uses equalsAsTerm for equals */
        final class LiteralWrapper {
            final private Literal l;
            public LiteralWrapper(Literal l) { this.l = l; }
            public int hashCode() { return l.hashCode(); }
            public boolean equals(Object o) { return o instanceof LiteralWrapper && l.equalsAsStructure(((LiteralWrapper)o).l); }
            public String toString() { return l.toString(); }
        }
    }
}
