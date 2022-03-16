package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.lib.drivers.PicoColorSensor;
import com.team1678.frc2022.lib.drivers.PicoColorSensor.RawColor;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private PicoColorSensor mPico0; //Color Sensor Closer to Intake TODO: check with electrical
    private PicoColorSensor mPico1; //Color Sensor Closer to Ejector TODO: check with electrical

    private Timer mHasBallTimer = new Timer();
    private Timer mEjectorTimer = new Timer();

    public ColorChoices mAllianceColor = ColorChoices.NONE;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE  
    }

    private ColorSensor() {
        mMatchedColor = ColorChoices.NONE;
        mPico0 = new PicoColorSensor();
        mPico1 = new PicoColorSensor();
        
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mPico0.start();
                mPico1.start();
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
        return mPeriodicIO.proximity_0 > Constants.ColorSensorConstants.kColorSensorThreshold;
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


    public void updateHasBall() {
        //update based off whether a ball enters into the system
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

    // update the color of the cargo we see
    public void updateMatchedColor() {
        if (mPeriodicIO.proximity_0 < Constants.ColorSensorConstants.kColorSensorThreshold) { 
            mMatchedColor = ColorChoices.NONE;
        } else {
            if (1.111 * mPeriodicIO.raw_color_0.red > 1.667 * mPeriodicIO.raw_color_0.blue) {
                mMatchedColor = ColorChoices.RED;
            } else if (1.111 * mPeriodicIO.raw_color_0.blue > 1.667 * mPeriodicIO.raw_color_0.red) {
                mMatchedColor = ColorChoices.BLUE;
            } else {
                mMatchedColor = ColorChoices.OTHER;
            }
        }
    }

    // update whether we want to eject or not
    public void updateWantsEject() {
        if (hasOppositeColor() && hasBall()) {
            mPeriodicIO.eject = true;
            mEjectorTimer.start();
        }

        if (mEjectorTimer.hasElapsed(Constants.IndexerConstants.kEjectDelay) || (hasCorrectColor() && hasBall())) {
            mPeriodicIO.eject = false;
            mEjectorTimer.reset();
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
        mPeriodicIO.sensor0Connected = mPico0.isSensor0Connected();
        mPeriodicIO.raw_color_0 = mPico0.getRawColor0();
        mPeriodicIO.proximity_0 = mPico0.getProximity0();

        mPeriodicIO.sensor1Connected = mPico1.isSensor1Connected();
        mPeriodicIO.raw_color_1 = mPico1.getRawColor1();
        mPeriodicIO.proximity_1 = mPico1.getProximity1();

        mPeriodicIO.timestamp = mPico0.getLastReadTimestampSeconds();

        updateHasBall();
        updateMatchedColor();
        updateWantsEject();
    }

    @Override 
    public synchronized void writePeriodicOutputs() {
        
    }

    //subystem getters
    public double getDetectedRValue0() {
        if (mPeriodicIO.raw_color_0 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_0.red;
    }
    public double getDetectedGValue0() {
        if (mPeriodicIO.raw_color_0 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_0.green;
    }

    public double getDetectedBValue0() {
        if (mPeriodicIO.raw_color_0 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_0.blue;
    }

    public double getDistance0() {
        return mPeriodicIO.proximity_0;
    }

    public double getDetectedRValue1() {
        if (mPeriodicIO.raw_color_1 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_1.red;
    }
    public double getDetectedGValue1() {
        if (mPeriodicIO.raw_color_1 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_1.green;
    }

    public double getDetectedBValue1() {
        if (mPeriodicIO.raw_color_1 == null) {
            return 0;
        }
        return mPeriodicIO.raw_color_1.blue;
    }

    public double getDistance1() {
        return mPeriodicIO.proximity_1;
    }

    public String getAllianceColor() {
        return mAllianceColor.toString();
    }

    public String getMatchedColor() {
        return mMatchedColor.toString();
    }

    public boolean getSensor0() {
        return mPeriodicIO.sensor0Connected;
    }

    public boolean getSensor1() {
        return mPeriodicIO.sensor1Connected;
    }

    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class PeriodicIO {
        // INPUTS
        public RawColor raw_color_0;
        public int proximity_0;
        public boolean sensor0Connected;

        public RawColor raw_color_1;
        public int proximity_1;
        public boolean sensor1Connected;

        // OUTPUTS
        public boolean has_ball;
        public boolean eject;
        public double timestamp;
    }
    
}
