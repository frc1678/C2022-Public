package com.team1678.frc2022.shuffleboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    protected NetworkTableEntry createNumberEntry(String name) {
        return mTab.add(name, 0.0).withSize(2, 1).getEntry();
    }

    protected NetworkTableEntry createStringEntry(String name) {
        return mTab.add(name, "").withSize(2, 1).getEntry();
    }

    public abstract void update();

    /* Truncates number to 2 decimal places for cleaner numbers */
    protected double truncate(double number) {
        return Math.floor(number * 100) / 100;
    }
    
    public ShuffleboardTab getTab() {
        return mTab;
    }
}
