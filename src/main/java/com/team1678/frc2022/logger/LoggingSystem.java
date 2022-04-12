package com.team1678.frc2022.logger;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.loops.ILooper;

import java.util.ArrayList;
import java.util.TimeZone;
import java.io.File;

import java.io.FileWriter;
import java.sql.Date;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class LoggingSystem {
    private static LoggingSystem mInstance; 
    //  set original directory path. Will be added to in LoggingSystem() when new directories are created inside /home/lvuser/logs
    public static String mRootDirectory = "/home/lvuser/logs";
    public static String mDirectory = mRootDirectory;
    ArrayList<ILoggable> loggableItems = new ArrayList<ILoggable>();

    ArrayList<FileWriter> loggableFiles = new ArrayList<FileWriter>();

    //  Use Boolean to decide whether or not to start logging
    Boolean log = true;

    /* 
        Create a for loop that goes over all the current files and subdirectories in mDirectories.
        If the directory is empty (when the max number is 0), start a new subdirectory at 1.
        Whenever the logging system reboots, function will scan over all the existing files and subdirectories and find the largest one.
        New subdirectory is created by adding one (1) to the max file number.
    */

    private LoggingSystem() {
        if (log == true) {
        //  Creates a new directory every time the robot is turned on (regardless of enabling/disabling)
        LogDirectory();
        }
    }

    public  void LogDirectory() {
        if (log == true) {
            File Directory = new File(mDirectory);
            Integer maxNum = 0;
            if  (! Directory.isDirectory()) {
                Directory.mkdir();
            }
            
            for (final File directoryEntry : Directory.listFiles()) {
                try {
                    if (directoryEntry.isDirectory()) {
                        String directory_name = directoryEntry.getName();
                        int char_index = directory_name.indexOf("_");
                        int Num = Integer.parseInt(directory_name.substring(0, char_index));
                        if (Num > maxNum) {
                            maxNum = Num;
                        }
                    } 
                } catch (Exception e) {
                    //  Files that are not numbers are expected and ignored
                }
            }
            maxNum++;

            // get system time in milliseconds and convert to datetime in PST time zone
            long milliSec = System.currentTimeMillis();
            Date res = new Date(milliSec);
            DateFormat sdf = new SimpleDateFormat("yy-MM-dd-HH-mm-ss");
            sdf.setTimeZone(TimeZone.getTimeZone("PST"));

            // format time in datetime and add to file name
            mDirectory = mRootDirectory + "/" + maxNum.toString() + "_" + sdf.format(res);

            File newDirectory = new File(mDirectory);
            newDirectory.mkdir();
        }
    }

    public synchronized static LoggingSystem getInstance() {
        if (mInstance == null) {
            mInstance = new LoggingSystem();
        }
        return mInstance; 
    }

    //  start function that opens file
    public void register(ILoggable newLoggable, String fileName) {
        if (log == true) {
            FileWriter fileWriter = null;
            try {
                fileWriter = new FileWriter(mDirectory + "/" + fileName);
            } catch (Exception e) {
                System.err.println("Couldn't register new file" + fileName);
            }
            ArrayList<String> itemNames = newLoggable.getItemNames();
            loggableFiles.add(fileWriter);
            //  Write names to file
            try {
                for (int h=0; h < itemNames.size(); h++) {
                    fileWriter.write(itemNames.get(h));
                    if (h != itemNames.size() - 1) {
                        fileWriter.write(",");
                    }
                }
                fileWriter.write("\n");
                //  Adding Loggable to loggableItems list
                loggableItems.add(newLoggable);
            } catch (Exception e) {
                System.err.println("Couldn't write to file");
            }
        }
    }

    //  Logging Function
    //  gets called when main begins logging
    void Log() {
        if (log == true) {
        try{
            for (int i=0; i < loggableItems.size(); i++) {
               ArrayList<ArrayList<Number>> items = loggableItems.get(i).getItems();
               //  get object fileWriter from the list 
               FileWriter fileWriter = loggableFiles.get(i);
               //  write to files
               for (int j=0; j < items.size(); j++) {
                   ArrayList<Number> data = items.get(j);
                   for (int m=0; m < data.size(); m++){
                        fileWriter.write(data.get(m).toString());
                        if (m != data.size()-1){
                            fileWriter.write(",");
                        }
                    }
                    fileWriter.write("\n");
                }
            }
        } catch (Exception e) {
            System.err.println("Couldn't get object and/or log it");
        }
    }
    }

    //  Close Logging System
    void Close() {
        if (log = true) {
            try {
                //  Get final logs
                Log();
                //  Close files 
                for (int i=0; i< loggableFiles.size(); i++) {
                    FileWriter fileWriter = loggableFiles.get(i);
                    fileWriter.close();
                }
            } catch (Exception e) {
                System.err.println("Couldn't close file");
            }
        }
    }

    public void registerLoops(ILooper looper) {
        looper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }
            @Override 
            public void onLoop(double timestamp) {
                if (log == true) {
                Log();
                }
            }
            @Override 
            public void onStop(double timestamp) {
            }
        });
    }
}
