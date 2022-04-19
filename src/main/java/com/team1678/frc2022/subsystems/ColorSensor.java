package com.team1678.frc2022.subsystems;

import java.util.ArrayList;

import com.lib.drivers.PicoColorSensor;
import com.lib.drivers.PicoColorSensor.RawColor;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.Constants.ColorSensorConstants;
import com.team1678.frc2022.logger.LogStorage;
import com.team1678.frc2022.logger.LoggingSystem;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team254.lib.util.Util;

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
    
    // logger
    LogStorage<PeriodicIO> mStorage = null;

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private PicoColorSensor mPico;

    private final DigitalInput mForwardBreak;
    private boolean mSawBall = false;

    private Timer mHasBallTimer = new Timer();
    private Timer mEjectorTimer = new Timer();

    public ColorChoices mAllianceColor = ColorChoices.NONE;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE  
    }

    private ColorSensor() {
        mMatchedColor = ColorChoices.NONE;
        mPico = new PicoColorSensor();

        mForwardBreak = new DigitalInput(Ports.getForwardBeamBreakPort());
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
                // send log data
                SendLog();
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
        return !Util.epsilonEquals(mPeriodicIO.color_ratio,
                                  1.0,
                                  Constants.ColorSensorConstants.kColorSensorRatioThreshold);
    }

    public boolean seesNewBall() {
        boolean newBall = false;
        if ((seesBall() && !mSawBall)) {
            newBall = true;
        }
        mSawBall = seesBall();
        return newBall;
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

    public void updateColorOffset() {
        if (mPeriodicIO.raw_color != null) {
            mPeriodicIO.color_offset = mPeriodicIO.raw_color.blue - mPeriodicIO.raw_color.red;
        }
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
            mAllianceColor = ColorChoices.NONE;
            DriverStation.reportError("No Alliance Color Detected", true);
        }
    }

    // update the color of the cargo we see
    public void updateMatchedColor() {
        if (Util.epsilonEquals(mPeriodicIO.color_ratio,
                               1.0,
                               Constants.ColorSensorConstants.kColorSensorRatioThreshold)) { 
            mMatchedColor = ColorChoices.NONE;
        } else {
            if (mPeriodicIO.color_ratio > 1.0) {
                mMatchedColor = ColorChoices.RED;
            } else if (mPeriodicIO.color_ratio < 1.0) {
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
        mPeriodicIO.sensor0Connected = mPico.isSensor0Connected();
        mPeriodicIO.raw_color = mPico.getRawColor0();
        mPeriodicIO.adjusted_blue = mPeriodicIO.raw_color.blue; // keep blue the same
        mPeriodicIO.adjusted_red = mPeriodicIO.raw_color.red + mPeriodicIO.color_offset /*Constants.ColorSensorConstants.kColorOffset*/;
        mPeriodicIO.color_ratio = (double) mPeriodicIO.adjusted_red / (double) mPeriodicIO.adjusted_blue;
        // mPeriodicIO.color_ratio = (double) mPeriodicIO.raw_color.red / (double) mPeriodicIO.raw_color.blue;
        mPeriodicIO.proximity = mPico.getProximity0();

        mPeriodicIO.timestamp = mPico.getLastReadTimestampSeconds();

        SmartDashboard.putNumber("color_ratio", mPeriodicIO.color_ratio);
        SmartDashboard.putNumber("color_offset", mPeriodicIO.color_offset);
        SmartDashboard.putNumber("adjusted_red", mPeriodicIO.adjusted_red);
        SmartDashboard.putNumber("adjusted_blue", mPeriodicIO.adjusted_blue);

        /*
        if (!getForwardBeamBreak()) {
            updateColorOffset();
        }
        */
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
    public ColorChoices getAllianceColor() {
        return mAllianceColor;
    }
    public String getMatchedColor() {
        return mMatchedColor.toString();
    }
    public boolean getForwardBeamBreak() {
        return !mForwardBreak.get();
    }
    public double getDistance() {
        return mPeriodicIO.proximity;
    }

    public boolean getSensor0() {
        return mPeriodicIO.sensor0Connected;
    }

    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;

        public RawColor raw_color;
        public double color_offset; // offset of blue - red
        public double adjusted_red;
        public double adjusted_blue;
        public double color_ratio; // ratio of red to blue color
        public int proximity;
        public boolean sensor0Connected;

        // OUTPUTS
        public boolean has_ball;
        public boolean eject;
    }

    @Override
    public void registerLogger(LoggingSystem LS) {
        SetupLog();
        LS.register(mStorage, "COLOR_SENSOR_LOGS.csv");
    }

    public void SetupLog() {
        mStorage = new LogStorage<PeriodicIO>();

        ArrayList<String> headers = new ArrayList<String>();
        headers.add("timestamp");

        headers.add("sensor_connected");

        headers.add("raw_red");
        headers.add("raw_blue");

        headers.add("adjusted_red");
        headers.add("adjusted_blue");
        headers.add("color_offset");

        headers.add("color_ratio");
        headers.add("proximity");

        headers.add("has_ball");
        headers.add("eject");

        mStorage.setHeaders(headers);
    }

    public void SendLog() {
        ArrayList<Number> items = new ArrayList<Number>();
        items.add(Timer.getFPGATimestamp());

        items.add(mPeriodicIO.sensor0Connected ? 1.0 : 0.0);

        items.add(mPeriodicIO.raw_color.red);
        items.add(mPeriodicIO.raw_color.blue);

        items.add(mPeriodicIO.adjusted_red);
        items.add(mPeriodicIO.adjusted_blue);
        items.add(mPeriodicIO.color_offset);

        items.add(mPeriodicIO.color_ratio);
        items.add(mPeriodicIO.proximity);

        items.add(mPeriodicIO.has_ball ? 1.0 : 0.0);
        items.add(mPeriodicIO.eject ? 1.0 : 0.0);

        // send data to logging storage
        mStorage.addData(items);
    }
    
}
