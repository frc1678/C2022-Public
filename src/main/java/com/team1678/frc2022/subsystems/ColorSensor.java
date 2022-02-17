package com.team1678.frc2022.subsystems;

import com.revrobotics.ColorMatch;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends Subsystem {

    private static ColorSensor mInstance;
    public static synchronized ColorSensor getInstance() {
        if (mInstance == null) {
            mInstance = new ColorSensor();
        }
        return mInstance;
    }

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private REVColorSensorV3Wrapper mColorSensor;

    private final ColorMatch mColorMatch = new ColorMatch();

    private Timer mEjectorTimer = new Timer();
    private boolean mEjectorRunning = false;

    public ColorChoices mAllianceColor;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER, NONE  
    }

    private ColorSensor() {
        if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Red) {
            mAllianceColor = ColorChoices.RED;
        } else if (edu.wpi.first.wpilibj.DriverStation.getAlliance() == Alliance.Blue){
            mAllianceColor = ColorChoices.BLUE;
        } else {
            DriverStation.reportError("No Alliance Color Detected", true);
        }
        mMatchedColor = ColorChoices.NONE;
        mColorSensor = new REVColorSensorV3Wrapper(I2C.Port.kOnboard);
        
        mColorMatch.addColorMatch(Constants.ColorSensorConstants.kRedColor);
        mColorMatch.addColorMatch(Constants.ColorSensorConstants.kBlueColor);

        mColorSensor.start();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
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

    // update the color of the cargo we see
    public void updateMatchedColor() {
        if (mPeriodicIO.distance < Constants.ColorSensorConstants.kColorSensorThreshold) { 
            mMatchedColor = ColorChoices.NONE;
        } else {
            if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kRedColor)) {
                mMatchedColor = ColorChoices.RED;
            } else if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kBlueColor)) {
                mMatchedColor = ColorChoices.BLUE;
            } else {
                mMatchedColor = ColorChoices.OTHER;
            }
        }
    }

    // update whether we want to eject or not
    public void updateWantsEject() {
        if (hasOppositeColor()) {
            mPeriodicIO.eject = true;

            mEjectorTimer.start();
            mEjectorRunning = true;
        }

        if (mEjectorTimer.hasElapsed(Constants.IndexerConstants.kEjectDelay) || hasCorrectColor()) {
            mPeriodicIO.eject = false;

            mEjectorTimer.reset();
            mEjectorRunning = false;
        }
    }

    public boolean wantsEject() {
        return mPeriodicIO.eject;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.rawColorSensorData = mColorSensor.getLatestReading();

        if (mPeriodicIO.rawColorSensorData != null) {
            mPeriodicIO.raw_color = mPeriodicIO.rawColorSensorData.color;
            mPeriodicIO.distance = mPeriodicIO.rawColorSensorData.distance;
            mPeriodicIO.matched_color = mColorMatch.matchClosestColor(mPeriodicIO.raw_color).color;
        } 

        updateMatchedColor();
        updateWantsEject();
    }

    @Override 
    public synchronized void writePeriodicOutputs() {
        
    }
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
    public String getMatchedColor() {
        return mMatchedColor.toString();
    }    
    public double getDistance() {
        return mPeriodicIO.distance;
    }

    public static class PeriodicIO {
        
        // INPUTS
        public ColorSensorData rawColorSensorData;
        public Color raw_color;
        public double distance;
        public Color matched_color;

        //OUTPUTS
        public boolean eject;
    }
    
}
