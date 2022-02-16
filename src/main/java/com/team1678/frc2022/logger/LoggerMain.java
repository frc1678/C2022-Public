package com.team1678.frc2022.logger;

public class LoggerMain {
    public static void main(String[] args) {
      LoggingSystem LS = LoggingSystem.getInstance();
       // creating a loggable object 
       ILoggable loggable = new TestLoggable();
       LS.register(loggable, "Test.csv");
       // telling system to log on a regular basis 
       for (int i = 0; i < 10; i++) {
         LS.Log();
         try {
           Thread.sleep(20);
         } catch (Exception e) {}
       }
    }
 }
 