package com.team1678.frc2022.logger;

import java.lang.reflect.Array;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.function.Supplier;

public class LogStorage<T> implements ILoggable {
    ArrayList<String> mColumns = new ArrayList<String>();
    ArrayList<ArrayList<Number>> mItems = new ArrayList<ArrayList<Number>>();

    private Supplier<T> supplier; 

    /*  Get Item:
            Creates an Array List of Array Lists of doubles for the logged data.
            Do a variable swap between a new array list and the old one to continue 
            logging and store old data.
            Returns the old data at the end of the function

        Get Item Names:
            Make columns to store the item names so that log files are in a neat grid type thing

            | timestamp | current | demand |  
            |           |         |        |
            |           |         |        |    Logged data goes under the columns
            |           |         |        |
            |           |         |        |
    */
    
    @Override
    public synchronized ArrayList<ArrayList<Number>> getItems() {
        ArrayList<ArrayList<Number>> items_tmp = mItems;
        mItems = new ArrayList<ArrayList<Number>>();
        return items_tmp;
    }

    @Override
    public ArrayList<String> getItemNames() {
        return mColumns;
    }

    public ArrayList<String> setHeadersFromClass(Class<T> typeClass) {
        Field[] mVars;
        mVars = typeClass.getFields();
        //  create array list to the names of the member variables collected
        mColumns.clear();
        //  For the member variables, get the variable  and add it to the array list
        for (Field variableName : mVars) {
            if (variableName.getType() == boolean[].class || variableName.getType() == double[].class || variableName.getType() == int[].class) {
                try {
                    T x = supplier.get(); //  there is no good name for this variable :(
                    Object obj = variableName.get(x);
                    int length = Array.getLength(obj);
                    for (int i = 0; i < length; i++) {
                        mColumns.add(variableName.getName() + Integer.toString(i));
                    }
                } catch (Exception e) {}
            } else {
                mColumns.add(variableName.getName());
            }
        }
        //  Log item names from array list
        return mColumns;
    }

    public void setHeaders(ArrayList<String> columns) {
        mColumns = columns;
    }

    public synchronized void addData(ArrayList<Number> items) {
        mItems.add(items);
    }
}
