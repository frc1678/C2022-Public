package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.ColorSensor;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.lib.drivers.REVColorSensorV3Wrapper;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;

public class Ejector extends Subsystem{
    private static Ejector mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private REVColorSensorV3Wrapper mColorSensor;
    private Solenoid mSolenoid;
    private TalonFX mMaster;
    
    public ColorSensor mColorSensorThread = new ColorSensor(mColorSensor);
    private final ColorMatch mColorMatcher = new ColorMatch();

    public ColorChoices mAllianceColor;
    public ColorChoices mMathcedColor;

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
        mMathcedColor = ColorChoices.NONE;
        mColorSensor = new REVColorSensorV3Wrapper(I2C.Port.kOnboard);
        mSolenoid = new Solenoid(Ports.PCM, PneumaticsModuleType.CTREPCM, Ports.EJECTOR_SOLENOID_ID);

        mColorMatcher.addColorMatch(Constants.EjectorConstants.kRedBallColor);
        mColorMatcher.addColorMatch(Constants.EjectorConstants.kBlueBallColor);
    }

    @Override
   public void registerEnabledLoops(ILooper enabledLooper) {
       enabledLooper.register(new Loop() {
           @Override
           public void onStart(double timestamp) {
                mColorSensorThread.start();
                mPeriodicIO.eject = false;
           }

           @Override
           public void onLoop(double timestamp) {
           }

           @Override
           public void onStop(double timestamp) {
                mColorSensorThread.stop();
           }
       });
   }

   @Override
   public synchronized void readPeriodicInputs() {

        if (mColorSensorThread.getColorSensorData() != null) {
            mPeriodicIO.rawColor = mColorSensorThread.getColorSensorData().color;
            mPeriodicIO.distance = mColorSensorThread.getColorSensorData().distance; 
            mPeriodicIO.matchedColor = mColorMatcher.matchClosestColor(mPeriodicIO.rawColor).color; 

            if (mPeriodicIO.distance < Constants.EjectorConstants.kColorSensorThreshold) {
                mPeriodicIO.eject = false;
                mMathcedColor = ColorChoices.NONE;
            } else {
                if (mPeriodicIO.matchedColor.equals(Constants.EjectorConstants.kRedBallColor)) {
                mMathcedColor = ColorChoices.RED;
                } else if (mPeriodicIO.matchedColor.equals(Constants.EjectorConstants.kBlueBallColor)) {
                mMathcedColor = ColorChoices.BLUE;
                }
                mPeriodicIO.eject = mMathcedColor == mAllianceColor;
            }
        }
   }

   @Override
   public void writePeriodicOutputs() {
        mSolenoid.set(mPeriodicIO.eject);
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand);
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

    public double getDetectedRValue() {
        return mPeriodicIO.rawColor.red;
    }

    public double getDetectedGValue() {
        return mPeriodicIO.rawColor.green;
    }

    public double getDetectedBValue() {
        return mPeriodicIO.rawColor.blue;
    }

    public double getDistance() {
        return mPeriodicIO.distance;
    }

    public String getMatchedColor() {
        return mMathcedColor.toString();
    }

    public static class PeriodicIO {
    
        //INPUTS
        public Color rawColor;
        public double distance;
        public Color matchedColor;

        //OUTPUTS
        public double demand;
        public boolean eject;

    }
    
}
