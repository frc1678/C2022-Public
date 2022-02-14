package com.team1678.frc2022;

import java.util.concurrent.BlockingQueue;

import com.team1678.frc2022.subsystems.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkTableInteractions extends Thread{

    private BlockingQueue<String> tableNameQueue; 
    private BlockingQueue<Double> txQueue;

    private NetworkTable mNetworkTable;
    
    public NetworkTableInteractions(NetworkTable mNetworkTable, BlockingQueue<String> tableNameQueue, BlockingQueue<Double> txQueue) {
        mNetworkTable = this.mNetworkTable;
        tableNameQueue = this.tableNameQueue;
        txQueue = this.txQueue;
    }

    public void run() {
        try {
            while (true) {
                System.out.println("Running NetworkTables Thread");
                mNetworkTable = NetworkTableInstance.getDefault().getTable(tableNameQueue.take());
                double tx = mNetworkTable.getEntry("tx").getDouble(0.0);
                txQueue.put(tx);

            }
        } catch (Exception e) {
            System.out.println("Exception is caught");
        }
    }
}
