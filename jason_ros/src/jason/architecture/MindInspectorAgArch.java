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

package jason.architecture;

import jason.NoValueException;
import jason.asSyntax.ASSyntax;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.Structure;
import jason.util.asl2html;
import jason.util.asl2xml;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Toolkit;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTextPane;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.w3c.dom.Document;

/**
 * ArArch that displays the mind state of the agent
 */
public class MindInspectorAgArch extends AgArch {
    
    // variables for mind inspector
    protected boolean hasMindInspectorByCycle = false;
    protected int     updateInterval = 0;
    protected static JFrame       mindInspectorFrame = null;
    protected static JTabbedPane  mindInspectorTab = null;
    protected        JTextPane    mindInspectorPanel = null;
    protected        JSlider      mindInspectorHistorySlider = null;
    protected        JCheckBox    mindInspectorFreeze = null;
    protected        List<String> mindInspectorHistory = null;    
    protected        asl2xml      mindInspectorTransformer = null;

    protected        String       mindInspectorDirectory;

    @Override
    public void init() {
        setupMindInspector(getTS().getSettings().getUserParameter("mindinspector"));
    }

    /**
     * A call-back method called by the infrastructure tier 
     * when the agent is about to be killed.
     */
    @Override
    public void stop() {
        if (mindInspectorFrame != null)
            mindInspectorFrame.dispose();
        super.stop();
    }

    @Override
    public void reasoningCycleStarting() {
        if (hasMindInspectorByCycle)
            updateMindInspector();
        super.reasoningCycleStarting();
    }
    
    
    
    /**
     *    process the mindinspector parameter used in the agent option in .mas2j project.
     *    E.g. agents bob x.asl [mindinspector="gui(cycle,html)"];
     *    
     *    General syntax of the parameter:
     *    [gui|file] ( [ cycle|number ] , [xml,html] [, history | directory] ) 
     */
    protected void setupMindInspector(String configuration) {
        Structure sConf = null;
        try {
            sConf = ASSyntax.parseStructure(configuration);
        } catch (Exception e) {
            getTS().getLogger().warning("The mindinspector argument does not parse as a predicate! "+configuration+" -- error: "+e);
            return;
        }
        
        // get the frequency of updates
        hasMindInspectorByCycle = sConf.getTerm(0).toString().equals("cycle");

        if (! hasMindInspectorByCycle) {
            try {
                updateInterval = (int)((NumberTerm)sConf.getTerm(0)).solve();
            } catch (NoValueException e1) {
                e1.printStackTrace();
            }
            new Thread("update agent mind inspector") {
                public void run() {
                    try {
                        while (isRunning()) {
                            Thread.sleep(updateInterval);
                            updateMindInspector();
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                };
            }.start();
        }
                
        if (sConf.getFunctor().equals("gui")) {
            createGUIMindInspector(sConf);
        } else if (sConf.getFunctor().equals("file")) {
            createFileMindInspector(sConf);            
        }
    }
    
    private void createGUIMindInspector(Structure sConf) {
        // assume html output
        String format = "text/html";

        if (mindInspectorFrame == null) { // Initiate the common window
            mindInspectorFrame = new JFrame("Mind Inspector");
            mindInspectorTab   = new JTabbedPane();
            mindInspectorFrame.getContentPane().add(mindInspectorTab);
            Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
            mindInspectorFrame.setBounds(100, 200, (int)((screenSize.width-100)*0.7), (int)((screenSize.height-100)*0.9));
            mindInspectorFrame.setVisible(true);
        }
            
        mindInspectorPanel = new JTextPane();
        mindInspectorPanel.setEditable(false);
        mindInspectorPanel.setContentType(format);

        // get history
        boolean hasHistory = sConf.getArity() == 3 && sConf.getTerm(2).toString().equals("history");
        if (! hasHistory) {
            mindInspectorTab.add(getAgName(), new JScrollPane(mindInspectorPanel));
        } else {
            mindInspectorHistory = new ArrayList<String>();
            JPanel pHistory = new JPanel(new BorderLayout());//new FlowLayout(FlowLayout.CENTER));
            pHistory.setBorder(BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(), "Agent History", TitledBorder.LEFT, TitledBorder.TOP));
            mindInspectorHistorySlider = new JSlider();
            mindInspectorHistorySlider.setMaximum(1);
            mindInspectorHistorySlider.setMinimum(0);
            mindInspectorHistorySlider.setValue(0);
            mindInspectorHistorySlider.setPaintTicks(true);
            mindInspectorHistorySlider.setPaintLabels(true);
            mindInspectorHistorySlider.setMajorTickSpacing(10);
            mindInspectorHistorySlider.setMinorTickSpacing(1);
            setupSlider();
            mindInspectorHistorySlider.addChangeListener(new ChangeListener() {
                public void stateChanged(ChangeEvent e) {
                    try {
                        int c = (int)mindInspectorHistorySlider.getValue();
                        mindInspectorPanel.setText(mindInspectorHistory.get(c));
                    } catch (Exception e2) {                        }
                }
            });
            pHistory.add(BorderLayout.CENTER, mindInspectorHistorySlider);
            
            mindInspectorFreeze = new JCheckBox();
            JPanel pf = new JPanel(new FlowLayout());
            pf.add(mindInspectorFreeze);
            pf.add(new JLabel("freeze"));
            pHistory.add(BorderLayout.EAST, pf);
            
            JPanel pAg = new JPanel(new BorderLayout());
            pAg.add(BorderLayout.CENTER, new JScrollPane(mindInspectorPanel));
            pAg.add(BorderLayout.SOUTH, pHistory);
            mindInspectorTab.add(getAgName(), pAg);
        }
        
        if (format.equals("text/html")) {
            mindInspectorTransformer = new asl2html("/xml/agInspection.xsl");
        }
    }
    
    
    private void setupSlider() {
        int size = mindInspectorHistory.size()-1;
        if (size < 0)
            return;
        
        Hashtable<Integer,Component> labelTable = new Hashtable<Integer,Component>();
        labelTable.put( 0, new JLabel("mind 0") );
        labelTable.put( size, new JLabel("mind "+size) );
        mindInspectorHistorySlider.setLabelTable( labelTable );
        mindInspectorHistorySlider.setMaximum(size);
        //mindInspectorHistorySlider.setValue(size);
    }

    
    private void createFileMindInspector(Structure sConf) {
        if (sConf.getArity() <= 2)
            mindInspectorDirectory = "log";
        else
            mindInspectorDirectory = sConf.getTerm(2).toString();
        
        // assume xml output
        mindInspectorTransformer = new asl2xml();
        
        // create directories
        mindInspectorDirectory += "/"+getAgName();
        File dirmind = new File(mindInspectorDirectory); 
        if (!dirmind.exists()) // create agent dir
            dirmind.mkdirs();

        // create a directory for this execution
        int c = 0;
        String d = mindInspectorDirectory+"/run-"+c;
        while (new File(d).exists()) {
            d = mindInspectorDirectory+"/run-"+(c++);
        }
        mindInspectorDirectory = d;
        new File(mindInspectorDirectory).mkdirs();
    }

    
    private String previousText = "";
    private int    fileCounter = 0;
    protected void updateMindInspector() {
        try {
            Document agState = getTS().getAg().getAgState(); // the XML representation of the agent's mind
            String sMind = mindInspectorTransformer.transform(agState); // transform to HTML
            if (sMind.equals(previousText)) 
                return; // nothing to log
            previousText = sMind;

            if (mindInspectorPanel != null) { // output on GUI
                if (mindInspectorFreeze == null || !mindInspectorFreeze.isSelected()) {
                    mindInspectorPanel.setText(sMind); // show the HTML in the screen
                }
                if (mindInspectorHistory != null) {
                    mindInspectorHistory.add(sMind);
                    setupSlider(); 
                    mindInspectorHistorySlider.setValue(mindInspectorHistory.size()-1);
                }
            } else if (mindInspectorDirectory != null) { // output on file
                String filename = String.format("%6d.xml",fileCounter++).replaceAll(" ","0");
                FileWriter outmind = new FileWriter(new File(mindInspectorDirectory+"/"+filename));
                outmind.write(sMind);
                outmind.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
