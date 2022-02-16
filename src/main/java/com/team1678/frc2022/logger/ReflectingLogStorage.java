/*  While using the reflective system to implement the logging system in subsystems is the cleanest 
    way to approach the implementation, IT IS 3X SLOWER than implementation by hand (about 300 milisec 
    rather than 90 milisec)
*/ 

package com.team1678.frc2022.logger;
import java.util.ArrayList;
import java.lang.reflect.Field;

public class ReflectingLogStorage<T> extends LogStorage<T> {
    private Field[] mVars;
    ArrayList<Number> newRow = new ArrayList<Number>();

    public ReflectingLogStorage(Class<T> typeClass) {
        //  getting member variables
        mVars = typeClass.getFields();
    }
    public ArrayList<String> getItemNames() {
        //  create array list to the names of the member variables collected
        ArrayList<String> getItemNames = new ArrayList<String>();
            //  for the member variables, get the variable  and add it to the array list
            for (Field variableName : mVars) {
                getItemNames.add(variableName.getName());
            }
        //  log item names from array list
        return getItemNames;
    }
    //  collect data from the variables and store in a new array list
    public void add(T classData) {
        newRow.clear();
        for (Field variableName : mVars) {
            try {
                //  add data from the variables and make sure they're doubles to add them to the array list 
                newRow.add(((Number)variableName.get(classData)).doubleValue());
            } catch (Exception e) {
                System.err.print("Unable to add variable name data to new row as a double value!");
            }
        }
        addData(newRow);
    }
}
