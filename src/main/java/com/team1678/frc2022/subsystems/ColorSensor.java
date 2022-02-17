package com.team1678.frc2022.subsystems;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import com.revrobotics.ColorMatch;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends Subsystem {

    private static ColorSensor mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private REVColorSensorV3Wrapper mColorSensor;

    private final ColorMatch mColorMatch = new ColorMatch();

    public ColorChoices mAllianceColor;
    public ColorChoices mMatchedColor;

    public enum ColorChoices {
        RED, BLUE, OTHER  
    }

    private ColorSensor() {
        mAllianceColor =  Constants.isRedAlliance ? ColorChoices.RED : ColorChoices.BLUE;
        mMatchedColor = ColorChoices.OTHER;
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

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.rawColorSensorData = mColorSensor.getLatestReading();

        if (mPeriodicIO.rawColorSensorData != null) {
            mPeriodicIO.raw_color = mPeriodicIO.rawColorSensorData.color;
            mPeriodicIO.distance = mPeriodicIO.rawColorSensorData.distance;
            mPeriodicIO.matched_color = mColorMatch.matchClosestColor(mPeriodicIO.raw_color).color;
        } 

        if (mPeriodicIO.distance < Constants.ColorSensorConstants.kColorSensorThreshold) { 
            mMatchedColor = ColorChoices.OTHER;
            mPeriodicIO.outtake = false;
        } else {

            if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kRedColor)) {
                mMatchedColor = ColorChoices.RED;
            } else if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kBlueColor)) {
                mMatchedColor = ColorChoices.BLUE;
            }
                mPeriodicIO.outtake = mMatchedColor == mAllianceColor;
        }

    }

    @Override 
    public synchronized void writePeriodicOutputs() {
        
    }

    public static ColorSensor getInstance() {
        return null;
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

    public boolean getOuttake() {
        return mPeriodicIO.outtake;
    }   

    public static class PeriodicIO {
        
        // INPUTS
        public ColorSensorData rawColorSensorData;
        public Color raw_color;
        public double distance;
        public Color matched_color;

        //OUTPUTS
        public boolean outtake;
    }
    
}