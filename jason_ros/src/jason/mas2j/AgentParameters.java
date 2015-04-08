package jason.mas2j;

import jason.architecture.AgArch;
import jason.asSemantics.Agent;
import jason.asSyntax.directives.Include;
import jason.bb.DefaultBeliefBase;
import jason.runtime.Settings;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

/** 
 * represents the agent declaration in the MAS2J project file. 
 * The project parser creates this object while parsing.
 * 
 * @author jomi
 */
public class AgentParameters {
    public String                name      = null;
    public File                  asSource  = null;
    public ClassParameters       agClass   = null;
    public ClassParameters       bbClass   = null;
    private List<ClassParameters> archClasses = new ArrayList<ClassParameters>();
    public int                   qty       = 1;
    private Map<String, String>  options   = null;
    private String               host      = null;
    
    public AgentParameters() {
        setupDefault();
    }
    
    public String toString() {
        return getAsInMASProject();
    }
    
    public void setupDefault() {
        if (agClass == null) {
            agClass = new ClassParameters(jason.asSemantics.Agent.class.getName());
        }
        if (bbClass == null) {
            bbClass = new ClassParameters(DefaultBeliefBase.class.getName());
        }        
        
    }
    
    /** fix source of the asl code based on aslsourcepath, also considers code from a jar file (if urlPrefix is not null) */
    public boolean fixSrc(List<String> srcpath, String urlPrefix) {
        String r = Include.checkPathAndFixWithSourcePath(asSource.toString(), srcpath, urlPrefix);
        if (r != null) {
            asSource = new File(r);
            return true;
        } else {
            return false;
        }
    }
    
    public ClassParameters getBBClass() {
        return bbClass;
    }
    
    public void setHost(String h) {
        if (h.startsWith("\""))
            host = h.substring(1,h.length()-1);
        else 
            host = h;
    }
    public String getHost()       { 
        return host; 
    }
    
    
    public void setAgClass(String c) {
        if (c != null) agClass = new ClassParameters(c);
    }
    
    public void addArchClass(String... cs) {
        for (String c: cs)
            archClasses.add(new ClassParameters(c));        
    }
    public void addArchClass(Collection<String> cs) {
        for (String c: cs)
            archClasses.add(new ClassParameters(c));        
    }
    public void addArchClass(ClassParameters... cps) {
        for (ClassParameters c: cps)
            archClasses.add(c);        
    }
    public void insertArchClass(ClassParameters... cps) {
        for (ClassParameters c: cps)
            archClasses.add(0,c);        
    }
    /** gets a list of all custom arch classes defined in the jason project */
    public List<String> getAgArchClasses() {
        List<String> all = new ArrayList<String>();
        for (ClassParameters c: archClasses) {
            all.add(c.getClassName());
        }
        return all;
    }    
    
    
    public void setBB(ClassParameters c) {
        if (c != null) bbClass = c;        
    }
    
    public void setOptions(Map<String,String> m) {
        for (String k: m.keySet())
            addOption(k, m.get(k));
    }
    
    public void addOption(String k, String vl) {
        if (options == null)
            options = new HashMap<String, String>();
        options.put(k, vl);
    }
    public String getOption(String key) {
        if (options == null)
            return null;
        else
            return options.get(key);
    }
    public Map<String,String> getOptions() {
        return options;
    }
    
    public String getAsInMASProject() {
        StringBuilder s = new StringBuilder(name+" ");
        if (asSource != null && !asSource.getName().startsWith(name)) {
            s.append(asSource+" ");
        }
        if (options != null && !options.isEmpty()) {
            s.append("[");
            Iterator<String> i = options.keySet().iterator();
            while (i.hasNext()) {
                String k = i.next();
                s.append(k+"="+options.get(k));
                if (i.hasNext()) {
                    s.append(", ");
                }
            }
            s.append("] ");
        }
        for (ClassParameters c: archClasses) {
            if (c.getClassName().length() > 0 && !c.getClassName().equals(AgArch.class.getName())) {
                s.append("agentArchClass "+c+" ");
            }            
        }
        if (agClass != null && agClass.getClassName().length() > 0 && !agClass.getClassName().equals(Agent.class.getName())) {
            s.append("agentClass "+agClass+" ");
        }
        if (bbClass != null && bbClass.getClassName().length() > 0 && !bbClass.getClassName().equals(DefaultBeliefBase.class.getName())) {
            s.append("beliefBaseClass "+bbClass+" ");
        }
        if (qty > 1) {
            s.append("#"+qty+" ");
        }
        if (host != null && host.length() > 0) {
            s.append("at \""+host+"\"");
        }
        return s.toString().trim() + ";";
    }

    public Settings getAsSetts(boolean debug, boolean forceSync) {
        Settings stts = new Settings();
        if (options != null) {
            String s = ""; String v = "";
            for (String key: options.keySet()) {
                s += v + key + "=" + options.get(key);
                v = ",";
            }
            if (s.length() > 0) {
                stts.setOptions("["+s+"]");
            }
        }
        if (debug) {
            stts.setVerbose(2);
        }
        
        if (forceSync || debug) {
            stts.setSync(true);
        }
        
        return stts;
    }
    
    public String getAgName() {
        return name;
    }
}
