/*test interface*/

//  Just testing that array list concepts and ILoggable work

package com.team1678.frc2022.logger; 

import java.util.ArrayList;

public class TestLoggable implements ILoggable {
    public ArrayList<ArrayList<Number>> getItems() {
        ArrayList<ArrayList<Number>> arrlist = new ArrayList<ArrayList<Number>>();
        ArrayList<Number> arr = new ArrayList<Number>();
        arr.add(1.0);
        arr.add(2.0);
        arr.add(3.0);
        arrlist.add(arr);
        return(arrlist);
    }
    public ArrayList<String> getItemNames() {
        ArrayList<String> ar = new ArrayList<String>();
        ar.add("var3");
        ar.add("var2");
        ar.add("var1");
        return(ar);
    }
}