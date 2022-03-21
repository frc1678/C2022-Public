package com.team1678.frc2022.subsystems;
 
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.lib.drivers.PicoColorSensor;
import com.team1678.frc2022.lib.drivers.PicoColorSensor.RawColor;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.util.MovingAverage;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class ColorSensor extends Subsystem {

    private static ColorSensor mInstance;
    public static synchronized ColorSensor getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensor();
        }
        return mInstance;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private PicoColorSensor mPico;

    private final DigitalInput mForwardBreak;

    private Timer mHasBallTimer = new Timer();
    private Timer mEjectTimer = new Timer();

    public ColorChoices mAllianceColor = ColorChoices.NONE;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE  
    }

    private ColorSensor() {
        mMatchedColor = ColorChoices.NONE;
        mPico = new PicoColorSensor();

        mForwardBreak = new DigitalInput(Ports.FORWARD_BEAM_BREAK);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mPico.start();
            }
 
            @Override
            public void onLoop(double timestamp) {
            }
 
            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    protected void start() {
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    // check if we see a ball
    public boolean seesBall() {
        return getForwardBeamBreak();
    }

    // check if we have the right color
    public boolean hasCorrectColor() {
        return mMatchedColor == mAllianceColor;
    }

    // check if we have the opposite color
    public boolean hasOppositeColor() {
        return !hasCorrectColor()
                    && (mMatchedColor != ColorChoices.OTHER)
                    && (mMatchedColor != ColorChoices.NONE);
    }

    // check if we have a ball
    public void updateHasBall() {
        if (seesBall()) {
            // reset the timer if we see another ball
            if (mPeriodicIO.has_ball) {
                mHasBallTimer.reset();
            }

            mPeriodicIO.has_ball = true;
            mHasBallTimer.start();
        }

        // if we don't see a ball and the threshold time for having a ball has passed, we don't have a ball anymore
        if (!seesBall() && mHasBallTimer.hasElapsed(Constants.ColorSensorConstants.kTimeWithBall)) {
            mHasBallTimer.reset();
            mPeriodicIO.has_ball = false;
        }
    }

    // update our alliance color
    // only should be updated in disabled periodic
    public void updateAllianceColor() {
        if (DriverStation.isDSAttached()) {
            if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
                mAllianceColor = ColorChoices.RED;
            } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue){
                mAllianceColor = ColorChoices.BLUE;
            }
        } else {
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    // update scaling of red and blue values for baseline when ball isn't seen
    public void updateBaselineColorScaling() {
        double color_diff = (double) mPeriodicIO.red_scaled - (double) mPeriodicIO.blue_scaled;

        mPeriodicIO.red_offset = -color_diff; // flip sign on offset
        mPeriodicIO.blue_offset = 0.0; // keep blue color raw so red can be scaled off of blue
    }

    // update the color of the cargo we see
    public void updateMatchedColor() {
        if (!hasBall()) { 
            mMatchedColor = ColorChoices.NONE;
        } else {
            double rb_diff = mPeriodicIO.red_final - mPeriodicIO.blue_final;
            // more red than blue
            if ((rb_diff > 0) && (Math.abs(rb_diff) > Constants.ColorSensorConstants.kColorDifferenceThreshold)) {
                mMatchedColor = ColorChoices.RED;
            // less red than blue
            } else if ((rb_diff < 0) && (Math.abs(rb_diff) > Constants.ColorSensorConstants.kColorDifferenceThreshold)) {
                mMatchedColor = ColorChoices.BLUE;
            } else {
                mMatchedColor = ColorChoices.OTHER;
            }
        }
        SmartDashboard.putNumber("Correct Color", hasCorrectColor() ? 1 : 0);
        SmartDashboard.putBoolean("Sees Ball", seesBall());

    }

    // update whether we want to eject or not
    public void updateWantsEject() {
        if (hasOppositeColor() && hasBall()) {
            mPeriodicIO.eject = true;
            mEjectTimer.start();
        }

        if (mEjectTimer.hasElapsed(Constants.IndexerConstants.kEjectDelay) || hasCorrectColor()) {
            mPeriodicIO.eject = false;
            mEjectTimer.reset();
        }
    }

    public boolean hasBall() {
        return mPeriodicIO.has_ball;
    }

    public boolean wantsEject() {
        return mPeriodicIO.eject;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.sensor0Connected = mPico.isSensor0Connected();
        mPeriodicIO.raw_color = mPico.getRawColor0();

        mPeriodicIO.timestamp = mPico.getLastReadTimestampSeconds();

        // update baseline color readings based off scaling when a ball isn't present
        /*
        if (!hasBall()) {
            updateBaselineColorScaling();
        }
        */
        
        // scale red and blue readings properly
        mPeriodicIO.blue_scaled = mPeriodicIO.raw_color.blue * Constants.ColorSensorConstants.kBlueFreqScaler;
        mPeriodicIO.red_scaled = mPeriodicIO.raw_color.red * Constants.ColorSensorConstants.kRedFreqScaler;

        mPeriodicIO.blue_final = mPeriodicIO.blue_scaled + mPeriodicIO.blue_offset;
        mPeriodicIO.red_final = mPeriodicIO.red_scaled + mPeriodicIO.red_offset;

        updateHasBall();
        updateMatchedColor();
        updateWantsEject();
    }

    @Override 
    public synchronized void writePeriodicOutputs() {
        
    }

    //subystem getters
    public double getDetectedRValue() {
        if (mPeriodicIO.raw_color == null) {
            return 0;
        }
        return mPeriodicIO.raw_color.red;
    }
    public double getDetectedGValue() {
        if (mPeriodicIO.raw_color == null) {
            return 0;
        }
        return mPeriodicIO.raw_color.green;
    }
    public double getDetectedBValue() {
        if (mPeriodicIO.raw_color == null) {
            return 0;
        }
        return mPeriodicIO.raw_color.blue;
    }
    public String getAllianceColor() {
        return mAllianceColor.toString();
    }
    public String getMatchedColor() {
        return mMatchedColor.toString();
    }    
    public boolean getForwardBeamBreak() {
        return !mForwardBreak.get();
    }

    public double getAdjustedRed() {
        return mPeriodicIO.red_final;
    }

    public double getAdjustedBlue() {
        return mPeriodicIO.blue_final;
    }

    public boolean getSensor0() {
        return mPeriodicIO.sensor0Connected;
    }

    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class PeriodicIO {
        // INPUTS
        public boolean sensor0Connected;

        public RawColor raw_color;

        public double red_offset;
        public double blue_offset;

        public double red_scaled;
        public double blue_scaled;

        public double red_final;
        public double blue_final;

        // OUTPUTS
        public boolean has_ball;
        public boolean eject;
        public double timestamp;
    }
    
}
