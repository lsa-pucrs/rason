package jason.stdlib;

import jason.*;
import jason.asSemantics.*;
import jason.asSyntax.*;

/**
<p>Internal action: <b><code>.nth</code></b>.

<p>Description: gets the nth term of a list.

<p>Parameters:<ul>
<li>+ index (integer): the position of the term, the first term is at position 0.<br/>
<li>+ list (list): the list where to get the term from.<br/>
<li>-/+ term (term): the term at position <i>index</i> in the <i>list</i>.<br/>
</ul>

<p>Examples:<ul>
<li> <code>.nth(0,[a,b,c],X)</code>: unifies <code>X</code> with <code>a</code>.
<li> <code>.nth(2,[a,b,c],X)</code>: unifies <code>X</code> with <code>c</code>.
<li> <code>.nth(0,[a,b,c],d)</code>: false.
<li> <code>.nth(0,[a,b,c],a)</code>: true.
<li> <code>.nth(5,[a,b,c],X)</code>: error.
</ul>

  @see jason.stdlib.concat
  @see jason.stdlib.delete
  @see jason.stdlib.length
  @see jason.stdlib.member
  @see jason.stdlib.sort
  @see jason.stdlib.max
  @see jason.stdlib.min
  @see jason.stdlib.reverse


  @see jason.stdlib.difference
  @see jason.stdlib.intersection
  @see jason.stdlib.union
*/

public class nth extends DefaultInternalAction {
    
    private static InternalAction singleton = null;
    public static InternalAction create() {
        if (singleton == null) 
            singleton = new nth();
        return singleton;
    }

    @Override public int getMinArgs() { return 3; }
    @Override public int getMaxArgs() { return 3; }

    @Override protected void checkArguments(Term[] args) throws JasonException {
        super.checkArguments(args); // check number of arguments
        if (!args[0].isNumeric()) {
            throw JasonException.createWrongArgument(this,"first argument should be numeric and not '"+args[0]+"'.");
        }
        if (!args[1].isList()) {
            throw JasonException.createWrongArgument(this,"second argument should be a list and not '"+args[1]+"'.");
        }
    }

    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        checkArguments(args);

        int index = (int)((NumberTerm)args[0]).solve();
        ListTerm list = (ListTerm)args[1];

        if (index < 0 || index >= list.size()) {
            throw new JasonException("nth: index "+index+" is out of bounds ("+list.size()+")");
        }
        Term element = list.get(index);

        return un.unifies(args[2], element);
    }
}
