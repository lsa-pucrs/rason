// ----------------------------------------------------------------------------
// Copyright (C) 2003 Rafael H. Bordini, Jomi F. Hubner, et al.
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
// 
// To contact the authors:
// http://www.inf.ufrgs.br/~bordini
// http://www.das.ufsc.br/~jomi
//
//----------------------------------------------------------------------------

package jason.asSyntax;

/**
 * Represents an unnamed variable '_'.
 * 
 * @author jomi
 */
public class UnnamedVar extends VarTerm {

    private static final long serialVersionUID = 1L;

    private static int varCont = 1;
    private int myId;
    //private boolean fromRename = false;
    
    public UnnamedVar() {
        super("_" + (varCont++));
        myId = varCont;
    }

    public UnnamedVar(String name) {
        super( name.length() == 1 ? "_" + (varCont++) : name);
        myId = varCont;
    }

    public UnnamedVar(int id) {
        super("_" + id);
        myId = id;
    }
    
    public static int getUniqueId() {
        return varCont++;
    }

    /*
    public void setFromMakeVarAnnon() {
        fromRename = true;
    }
    
    public boolean isFromMakeVarAnnon() {
        return fromRename;
    }
    */
    
    public Term clone() {
        /*if (hasValue()) {
            return getValue().clone();
        } else {*/
            UnnamedVar newv = new UnnamedVar(getFunctor());
            newv.myId = this.myId;
            //newv.fromRename = this.fromRename;
            if (hasAnnot())
                newv.addAnnots(this.getAnnots().cloneLT());
            return newv;
        //}
    }
    
    public int compareTo(Term t) {
        /*if (hasValue()) {
            return super.compareTo(t);
        } else */if (t instanceof UnnamedVar) {
            if (myId > ((UnnamedVar)t).myId)
                return 1;
            else if (myId < ((UnnamedVar)t).myId)
                return -1;
            else
                return 0;
        } else if (t instanceof VarTerm) {
            return 1;
        } else {
            return super.compareTo(t);
        }
    }

    @Override
    public boolean isUnnamedVar() {
        return true; //!hasValue();
    }
}
