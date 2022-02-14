package com.team1678.frc2022;

import java.util.HashMap;
import java.util.concurrent.BlockingQueue;

import com.team1678.frc2022.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableInteractions extends Thread{

    private BlockingQueue<Integer> inputQueue; 
    private BlockingQueue<HashMap> outputQueue;

    private NetworkTable mNetworkTable;
    
    public NetworkTableInteractions(NetworkTable mNetworkTable, BlockingQueue<Integer> inputQueue, BlockingQueue<HashMap> outputQueue) {
        mNetworkTable = this.mNetworkTable;
        inputQueue = this.inputQueue;
        outputQueue = this.outputQueue;
    }

    public void run() {
        try {
            while (true) {
                System.out.println("Running NetworkTables Thread");
                Integer input = inputQueue.take();
                HashMap<String, Double> outputMap = new HashMap<>(); 
                double tv = mNetworkTable.getEntry("tv").getDouble(0.0);
                outputMap.put("tv", tv);
                double tx = mNetworkTable.getEntry("tx").getDouble(0.0);
                outputMap.put("tx", tx);
                double ty = mNetworkTable.getEntry("ty").getDouble(0.0);
                outputMap.put("ty", ty);
                double tl = mNetworkTable.getEntry("tl").getDouble(0);
                outputMap.put("tl", tl);
                double ta = mNetworkTable.getEntry("ta").getDouble(0.0);
                outputMap.put("ta", ta);
                double ledMode = mNetworkTable.getEntry("ledMode").getDouble(1.0);
                outputMap.put("ledMode", ledMode);
                double pipeline = mNetworkTable.getEntry("pipeline").getDouble(0);
                outputMap.put("pipeline", pipeline);

                outputQueue.put(outputMap);

            }
        } catch (Exception e) {
            System.out.println("Exception is caught");
        }
    }
}
