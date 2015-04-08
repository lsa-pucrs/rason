package test;

import jason.asSemantics.Agent;
import jason.asSemantics.Unifier;
import jason.asSyntax.ASSyntax;
import jason.asSyntax.Literal;
import jason.asSyntax.LogExpr;
import jason.asSyntax.Rule;
import jason.asSyntax.Structure;
import jason.asSyntax.VarTerm;
import jason.asSyntax.parser.ParseException;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import junit.framework.TestCase;

/** JUnit test case for syntax package */
public class RuleTest extends TestCase {

    protected void setUp() throws Exception {
        super.setUp();
    }
 
    public void testLogCons() {
        Agent ag = new Agent();
        ag.initAg();

        ag.getBB().add(Literal.parseLiteral("a(10)"));
        ag.getBB().add(Literal.parseLiteral("a(20)"));
        ag.getBB().add(Literal.parseLiteral("a(30)"));
        ag.getBB().add(Literal.parseLiteral("b(20,10)"));
        ag.getBB().add(Literal.parseLiteral("c(x)"));
        ag.getBB().add(Literal.parseLiteral("c(y)"));
        ag.getBB().add(Literal.parseLiteral("c(20)"));

        // add r(X) :- a(X)
        Rule r = new Rule(Literal.parseLiteral("r(X)"), Literal.parseLiteral("a(X)"));
        ag.getBB().add(r);
        
        Iterator<Unifier> iun = Literal.parseLiteral("r(20)").logicalConsequence(ag, new Unifier());
        assertEquals(1,iteratorSize(iun));

        iun = Literal.parseLiteral("r(Y)").logicalConsequence(ag, new Unifier());
        assertEquals(3,iteratorSize(iun));

        // add v(X) :- a(X) & X > 15 | c(X)
        r = new Rule(Literal.parseLiteral("v(X)"), LogExpr.parseExpr("a(X) & X > 15 | c(X)"));  
        ag.getBB().add(r);

        iun = Literal.parseLiteral("v(30)").logicalConsequence(ag, new Unifier());
        assertEquals(1,iteratorSize(iun));

        iun = Literal.parseLiteral("v(20)").logicalConsequence(ag, new Unifier());
        assertEquals(2,iteratorSize(iun));

        iun = Literal.parseLiteral("v(A)").logicalConsequence(ag, new Unifier());
        //while (iun.hasNext()) {
        //    System.out.println(iun.next());
        //}
        assertEquals(5,iteratorSize(iun));

        // add s(X) :- r(X)
        r = new Rule(Literal.parseLiteral("s(X)"), LogExpr.parseExpr("r(X)"));
        ag.getBB().add(r);
        iun = Literal.parseLiteral("s(X)").logicalConsequence(ag, new Unifier());
        assertEquals(3,iteratorSize(iun));
        
        // add t(a) :- s(X)
        r = new Rule(Literal.parseLiteral("t(a)"), LogExpr.parseExpr("s(X)"));
        ag.getBB().add(r);
        
        iun = Literal.parseLiteral("t(X)").logicalConsequence(ag, new Unifier());
        assertEquals(3,iteratorSize(iun));
    }
    
    public void testAdd() throws ParseException {
        Agent ag = new Agent();
        ag.initAg();
        Literal r1 = ASSyntax.parseRule("price(_Service,X) :- .random(R) & X = (10*R)+100.");
        Literal r2 = (Literal)r1.capply(null);
        assertEquals(r1,r2);
    }

    public void testLogCons2() {
        Agent ag = new Agent();
        ag.initAg();
        
        Rule r = new Rule(Literal.parseLiteral("r([],a(X),b(X,4))"), Literal.parseLiteral("true"));
        ag.getBB().add(r);

        Iterator<Unifier> iun = Literal.parseLiteral("r([],a(20),X)").logicalConsequence(ag, new Unifier());
        //assertEquals(iun.next().get("X").toString(),"b(20,4)");
        assertTrue(iun.hasNext());
        Literal result = Literal.parseLiteral("r([],a(20),X)");
        Unifier u = iun.next();
        assertTrue(u.get("X").isStructure());
        assertEquals( ((Structure)u.get("X")).getArity(), 2);
        assertEquals( ((Structure)u.get("X")).getFunctor(), "b");
        assertEquals(result.capply(u), Literal.parseLiteral("r([],a(20),b(20,4))"));
        
        iun = Literal.parseLiteral("r([],a(20),b(X,Y))").logicalConsequence(ag, new Unifier());
        u = iun.next();
        assertEquals(u.get("X"), ASSyntax.createNumber(20));
        assertEquals(u.get("Y"), ASSyntax.createNumber(4));
    
    }    

    public void testLogConsRec() {
        Agent ag = new Agent();
        ag.initAg();
        
        // add 
        // min([],M,M).
        // min([C|T],V,M) :- C < V & min(T,C,M).
        // min([_|T],V,M) :- min(T,V,M).
        //ag.getBB().add(Literal.parseLiteral("min([],M,M)"));
        ag.getBB().add(new Rule(Literal.parseLiteral("min([],M,M)"), Literal.LTrue));
        ag.getBB().add(new Rule(Literal.parseLiteral("min([op(C)|T], op(V), M)"), 
                LogExpr.parseExpr("C < V & min(T,op(C),M)")));
        ag.getBB().add(new Rule(Literal.parseLiteral("min([op(C)|T], op(V), M)"), 
                LogExpr.parseExpr("C >= V & min(T,op(V),M)")));

        Iterator<Unifier> iun = Literal.parseLiteral("min([],op(20),op(M))").logicalConsequence(ag, new Unifier());
        assertTrue(iun.hasNext());
        Unifier u = iun.next();
        assertEquals(u.get("M").toString(),"20");

        Literal cons = Literal.parseLiteral("min([op(5),op(3),op(8),op(1),op(40)],op(1000),op(M))");
        Iterator<Literal> il = ag.getBB().getCandidateBeliefs(cons, null);
        assertEquals(3,iteratorSize(il));
        
        iun = cons.logicalConsequence(ag, new Unifier());
        u = iun.next();
        assertEquals((int)Double.parseDouble(u.get("M").toString()),1);
    }    
    
    public void testHasVar() {
        Rule r = new Rule(Literal.parseLiteral("a(X,Y)"), LogExpr.parseExpr("b(X) & c(Y,W) & d(Y,W,R)"));
        
        Map<VarTerm, Integer> c = new HashMap<VarTerm, Integer>();
        r.countVars(c);
        assertEquals(3,c.get(new VarTerm("Y")).intValue());
        
        assertEquals(1, r.getSingletonVars().size());
        assertEquals("[R]", r.getSingletonVars().toString());
        
        assertTrue(r.hasVar(new VarTerm("X"), null));
        assertTrue(r.hasVar(new VarTerm("W"), null));
        assertFalse(r.hasVar(new VarTerm("III"), null));        
    }
    
    
    @SuppressWarnings("unchecked")
    private int iteratorSize(Iterator i) {
        int c = 0;
        while (i.hasNext()) {
            i.next();
            c++;
        }
        return c;
    }
    
}
