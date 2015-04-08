// Internal action code for project tell-rule.mas2j

package rules;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.ListTerm;
import jason.asSyntax.ListTermImpl;
import jason.asSyntax.Literal;
import jason.asSyntax.StringTermImpl;
import jason.asSyntax.Term;

import java.util.Iterator;
import java.util.logging.Logger;

public class get_rules extends DefaultInternalAction {

    private Logger logger = Logger.getLogger("tell-rule.mas2j."+get_rules.class.getName());

    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        try {
            Literal pattern = (Literal)args[0];
            Iterator<Literal> i = ts.getAg().getBB().getCandidateBeliefs(pattern, un);
            ListTerm result = new ListTermImpl();
            while (i.hasNext()) {
                Literal l = i.next();
                if (l.isRule()) {
                    if (un.clone().unifies(pattern, l)) {
                        l = l.copy();
                        l.delSources();
                        result.add(new StringTermImpl(l.toString()));
                    }
                }
            }
            return un.unifies(args[1],result);
        } catch (Exception e) {
            logger.warning("Error in internal action 'get_rules'! "+e);
        }
        return false;
    }
}

