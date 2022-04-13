package com.team1678.frc2022.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BeamBreak {

    private boolean lastStatus;

    private boolean tripped;
    private boolean cleared;

    private final DigitalInput mBreak;

    public BeamBreak(int channel) {
        mBreak = new DigitalInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return !mBreak.get();
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}
