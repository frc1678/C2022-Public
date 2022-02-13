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

    BlockingQueue<Integer> commandQueue = new LinkedBlockingQueue<>(10);
    BlockingQueue<ColorSensorData> outputQueue = new LinkedBlockingQueue<>(10);

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

    public ColorSensor(REVColorSensorV3Wrapper mColorSensor2, BlockingQueue<Integer> commandQueue2,
            BlockingQueue<ColorSensorData> outputQueue2) {
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
        mColorSensor.getLatestReading();
    }

    @Override 
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.rawColorSensorData != null) {
            mPeriodicIO.raw_color = mPeriodicIO.rawColorSensorData.color;
            mPeriodicIO.matched_color = mColorMatch.matchClosestColor(mPeriodicIO.raw_color).color;
        } else {
            if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kRedColor)) {
                mMatchedColor = ColorChoices.RED;
            } else if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kBlueColor)) {
                mMatchedColor = ColorChoices.BLUE;
            } else {
                System.out.println("Invalid Color Detected");
            }
        }
    }

    public static ColorSensor getInstance() {
        return null;
    }
    
    public double getDetectedRValue() {
        if (mPeriodicIO.raw_color == null) {
            return 5;
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

    public ColorSensorData getLatestReading() {
        synchronized (mPeriodicIO.rawColorSensorData) {
            return mPeriodicIO.detected_color;
        }
    }

    public boolean getCorrectColor() {
        if (mPeriodicIO.red_alliance = Constants.isRedAlliance) {
            if(mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kRedColor)) {
                return mPeriodicIO.correct_color;
            } else {
                return !mPeriodicIO.correct_color;
            }
        } if (mPeriodicIO.blue_alliance != Constants.isRedAlliance) {
            if (mPeriodicIO.matched_color.equals(Constants.ColorSensorConstants.kBlueColor)) {
                return mPeriodicIO.correct_color;
            } else {
                return !mPeriodicIO.correct_color;
            }
        } else {
            return !mPeriodicIO.correct_color;
        }
    }

    public static class PeriodicIO {
        
        // INPUTS
        public ColorSensorData rawColorSensorData;
        public Color raw_color;
        public double distance;
        public Color matched_color;
        public ColorSensorData detected_color;
        public boolean correct_color;

        public boolean red_alliance;
        public boolean blue_alliance;

        //OUTPUTS

    }
    
}
