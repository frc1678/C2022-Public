package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.ILooper;

public class Shooter extends Subsystem {

    public void writeToLog() {
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public void stop() {
        // empty
    }

    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }

    public boolean checkSystem() {
        return true;
    }

    public boolean hasEmergency = false;
}   
