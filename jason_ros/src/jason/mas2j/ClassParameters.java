package jason.mas2j;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import jason.asSyntax.*;

/** 
 * Used to store class parameters in .mas2j file, e.g. 
 *      environment: Mars(a,b,c); 
 * this class stores 
 *   className  = Mars,
 *   parameters = {a,b,c}
 * 
 * @author jomi
 */
public class ClassParameters {
    private String className;
    private List<String> parameters = new ArrayList<String>();
    private String host;
    
    public ClassParameters() {}
    public ClassParameters(String className) {
        this.className = className;
    }
    public ClassParameters(Structure s) {
        className = s.getFunctor();
        if (s.getArity() > 0) {
            for (Term t: s.getTerms()) {
                parameters.add(t.toString());
            }
        }
    }
    
    public void setClassName(String cn) {
        className = cn;
    }
    public String getClassName() {
        return className;
    }
    

    public void addParameter(String s) {
        parameters.add(s);
    }
    public String getParameter(int index) {
        return parameters.get(index);
    }
    public String getParameter(String startWith) {
        for (String s: parameters) 
            if (s.startsWith(startWith))
                return s;
        return null;
    }
    public boolean hasParameter(String s) {
        return parameters.contains(s);
    }
    public boolean hasParameters() {
        return !parameters.isEmpty();
    }
    public String[] getParametersArray() {        
        String[] p = new String[parameters.size()];
        int i=0;
        for (String s: parameters) {
            p[i++] = removeQuotes(s);
        }
        return p;
    }
    /** returns parameters with space as separator */
    public String getParametersStr(String sep) {
        StringBuilder out = new StringBuilder();
        if (parameters.size() > 0) {
            Iterator<String> i = parameters.iterator();
            while (i.hasNext()) {
                out.append(i.next());
                if (i.hasNext()) out.append(sep);
            }
        }
        return out.toString();
        
    }

    public void setHost(String h) {
        if (h.startsWith("\""))
            host = h.substring(1,h.length()-1);
        else 
            host = h;
    }
    public String getHost() {
        return host;
    }
    
    public String toString() {
        StringBuilder out = new StringBuilder(className);
        if (parameters.size() > 0) {
            out.append("(");
            Iterator<String> i = parameters.iterator();
            while (i.hasNext()) {
                out.append(i.next());
                if (i.hasNext()) {
                    out.append(",");
                }
            }
            out.append(")");
        }
        return out.toString();
    }

    String removeQuotes(String s) {
        if (s.startsWith("\"") && s.endsWith("\"")) {
            return s.substring(1, s.length() - 1);
        } else {
            return s;
        }
    }
}
