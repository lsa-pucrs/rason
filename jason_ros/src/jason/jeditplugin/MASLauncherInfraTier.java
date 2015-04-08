package jason.jeditplugin;

import jason.mas2j.MAS2JProject;

/** 
 * Used by the Jason IDE to launch an MAS. Each infrastructure should
 * implements it.
 *
 * <p>The methods of this class are called in the following order:
 * <ul>
 * <li>setProject</li>
 * <li>setListener</li>
 * <li>writeScripts</li>
 * <li>stopMAS</li>
 * </ul>
 */
public interface MASLauncherInfraTier extends Runnable {

    /**
     * Sets the description of the project to be run.
     */
    public void setProject(MAS2JProject project);

    /** 
     * Sets the object that wants to be informed about the changes in
     * the execution state. This object normally is the JasonIDE, that
     * wants to change available buttons as the project is being
     * executed.
     */
    public void setListener(RunProjectListener listener);

    /**
     * Writes the script(s), normally Ant scripts, used to launch the
     * MAS.
     */
    public boolean writeScripts(boolean debug);

    /**
     * Stops the MAS execution.
     */
    public void stopMAS();
}
