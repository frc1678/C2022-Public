package com.team1678.frc2022;

import java.util.HashMap;
import java.util.concurrent.BlockingQueue;

import com.team1678.frc2022.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableInteractions extends Thread{

    private BlockingQueue<String> tableNameQueue; 
    private BlockingQueue<HashMap> outputQueue;

    private NetworkTable mNetworkTable;
    
    public NetworkTableInteractions(NetworkTable mNetworkTable, BlockingQueue<String> tableNameQueue, BlockingQueue<HashMap> outputQueue) {
        mNetworkTable = this.mNetworkTable;
        tableNameQueue = this.tableNameQueue;
        outputQueue = this.outputQueue;
    }

    public void run() {
        try {
            while (true) {
                System.out.println("Running NetworkTables Thread");
                mNetworkTable = NetworkTableInstance.getDefault().getTable(tableNameQueue.take());
                HashMap<String, Object> outputMap = new HashMap<>(); 
                double tx = mNetworkTable.getEntry("tx").getDouble(0.0);
                outputMap.put("tx", tx);
                double ty = mNetworkTable.getEntry("ty").getDouble(0.0);
                outputMap.put("ty", ty);
                double ta = mNetworkTable.getEntry("ta").getDouble(0.0);
                outputMap.put("ta", ta);
                outputQueue.put(outputMap);

            }
        } catch (Exception e) {
            System.out.println("Exception is caught");
        }
    }
}
