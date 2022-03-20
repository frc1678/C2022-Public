package com.team1678.frc2022.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak extends DigitalInput {

    private boolean lastTripped;
    private boolean lastCleared;
    

    public BeamBreak(int channel) {
        super(channel);
    }

    public boolean getTripped() {
        boolean tripped = false;
        if (this.get() && !lastTripped) {
            tripped = true;
        }
        lastTripped = this.get();
        return tripped;
    }

    public boolean getCleared() {
        boolean cleared = false;
        if (!this.get() && lastCleared) {
            cleared = true;
        }
        lastCleared = this.get();
        return cleared;
    }
    
}
