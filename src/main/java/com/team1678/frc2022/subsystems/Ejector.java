package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.ColorSensor;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper.ColorSensorData;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;

public class Ejector extends Subsystem {

    BlockingQueue<Integer> commandQueue = new LinkedBlockingQueue<>(10);
    BlockingQueue<ColorSensorData> outputQueue = new LinkedBlockingQueue<>(10);

    private static Ejector mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private REVColorSensorV3Wrapper mColorSensor;
    private Solenoid mSolenoid;
    
    //public ColorSensor mColorSensorThread = new ColorSensor(mColorSensor, commandQueue, outputQueue);
    private final ColorMatch mColorMatcher = new ColorMatch();

    public ColorChoices mAllianceColor;
    public ColorChoices mMatchedColor;

    public boolean mStopIndexer = false;

    public long detectTimestamp = 0;


    private final double kEjectTimeDelay = 2.0;
    private Timer mEjectTimer = new Timer();

    public enum ColorChoices {
        RED, BLUE, NONE
    }
    
    public static synchronized Ejector getInstance() {
        if (mInstance == null) {
            mInstance = new Ejector();
        }
        return mInstance;
    }

    private Ejector() {
        mAllianceColor = Constants.isRedAlliance ? ColorChoices.RED : ColorChoices.BLUE;
        mMatchedColor = ColorChoices.NONE;
        mColorSensor = new REVColorSensorV3Wrapper(I2C.Port.kOnboard);
        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.EJECTOR_SOLENOID_ID);

        mColorMatcher.addColorMatch(Constants.EjectorConstants.kRedBallColor);
        mColorMatcher.addColorMatch(Constants.EjectorConstants.kBlueBallColor);

        mColorSensor.start();
    }

    @Override
   public void registerEnabledLoops(ILooper enabledLooper) {
       enabledLooper.register(new Loop() {
           @Override
           public void onStart(double timestamp) {
                //mColorSensorThread.start();
                //mPeriodicIO.eject = false;
           }

           @Override
           public void onLoop(double timestamp) {
           }

           @Override
           public void onStop(double timestamp) {
                //mColorSensorThread.stop();
           }
       });
   }

   @Override
   public synchronized void readPeriodicInputs() {

    /*
        try {
            commandQueue.put(1);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }*/

   }

   @Override
   public void writePeriodicOutputs() {

        /* This should usually be in read periodic inputs — 
        only put into write periodic outputs to accomodate for timing of color sensor thread*/
        /*
        try { 
            mPeriodicIO.rawColorData = outputQueue.take();
            System.out.println(mPeriodicIO.rawColorData.timestamp);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }*/
        mPeriodicIO.rawColorData = mColorSensor.getLatestReading();
        System.out.print("Checking color data");

        if (mPeriodicIO.rawColorData != null) {
            mPeriodicIO.rawColor = mPeriodicIO.rawColorData.color;
            mPeriodicIO.distance = mPeriodicIO.rawColorData.distance; 
            mPeriodicIO.matchedColor = mColorMatcher.matchClosestColor(mPeriodicIO.rawColor).color; 

            if (mPeriodicIO.distance < Constants.EjectorConstants.kColorSensorThreshold) {
                if (mEjectTimer.hasElapsed(kEjectTimeDelay)) {
                    mEjectTimer.reset();
                    mPeriodicIO.eject = false;
                }
                mMatchedColor = ColorChoices.NONE;
            } else {
                if (mPeriodicIO.matchedColor.equals(Constants.EjectorConstants.kRedBallColor)) {
                mMatchedColor = ColorChoices.RED;
                } else if (mPeriodicIO.matchedColor.equals(Constants.EjectorConstants.kBlueBallColor)) {
                mMatchedColor = ColorChoices.BLUE;
                }

                //mPeriodicIO.eject = mMatchedColor == mAllianceColor;
                if (mMatchedColor == mAllianceColor && mPeriodicIO.eject == false) {
                    mEjectTimer.start();
                    mPeriodicIO.eject = true;
                    mStopIndexer = true;
                    detectTimestamp = System.currentTimeMillis();
                }

                if (mEjectTimer.hasElapsed(kEjectTimeDelay)) {
                    mEjectTimer.reset();
                    mPeriodicIO.eject = false;
                }

                if (mEjectTimer.hasElapsed(0.25)) {
                    mStopIndexer = false;
                }
            }
        } else {
            System.out.print("Null color data");
        }
        
        mSolenoid.set(mPeriodicIO.eject);
        SmartDashboard.putNumber("Eject Latency", (System.currentTimeMillis() - detectTimestamp) / 1000);
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

    public boolean getEject() {
        return mPeriodicIO.eject;
    }
    
    public boolean shouldStopHoppper() {
        return mStopIndexer;
    }

    public double getDetectedRValue() {
        if (mPeriodicIO.rawColor == null) {
            return 5;
        }
        return mPeriodicIO.rawColor.red;
    }

    public double getDetectedGValue() {
        if (mPeriodicIO.rawColor == null) {
            return 0;
        }
        return mPeriodicIO.rawColor.green;
    }

    public double getDetectedBValue() {
        if (mPeriodicIO.rawColor == null) {
            return 0;
        }
        return mPeriodicIO.rawColor.blue;
    }

    public double getDistance() {
        return mPeriodicIO.distance;
    }

    public String getMatchedColor() {
        return mMatchedColor.toString();
    }

    public static class PeriodicIO {
    
        //INPUTS
        public ColorSensorData rawColorData;
        public Color rawColor;
        public double distance;
        public Color matchedColor;

        //OUTPUTS
        public double demand;
        public boolean eject;

    }
    
}
