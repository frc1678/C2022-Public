package com.team1678.frc2022.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem {

    public CANdle mCandle = new CANdle(Ports.CANDLE);
    
    public State mState = State.DISABLED;
    public static LEDs mInstance;
    public double mLastTimestamp = Timer.getFPGATimestamp();

    public boolean mRising = false;
    public double mPhase = 0;
    public int mColorPhase = 0;

    public final Superstructure mSuperstructure = Superstructure.getInstance();
    public final Indexer mIndexer = Indexer.getInstance();

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public enum State {
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255, 20, 30, 4.0, 4.0, true),
        IDLE(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false),
        EMERGENCY(255, 0, 0, 0.3, 0.3, false),
        ONE_BALL(0.2, 0.2, false, new Color(0.5, 0.1, 0.1), new Color(0.1, 0.1, 0.5)),
        TWO_BALL(0.05, 0.05, false, new Color(0.5, 0.1, 0.1), new Color(0.1, 0.1, 0.5)),
        TARGET_VISIBLE(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false),
        SHOT_READY(255, 255, 0, 0.05, 0.05, false),
        WHITE(255, 255, 255, 10, 5, false),
        MERICA(0.3, 0, false, new Color(1, 1, 1), new Color(1, 0, 0), new Color(0, 0, 1)),
        CITRUS(0.5, 0.5, true, new Color(0.23, 0.83, 0.18), new Color(0, 1, 0)),
        RAINBOW(); // :)

        List<Color> colors = new ArrayList<Color>();
        double onTime, offTime;
        boolean breathing = false;
        boolean isCycleColors = false;

        /**
         * Creates a new state
         * @param r the red value
         * @param g the green value
         * @param b the blue value
         * @param onTime how long it should take for the color to turn on during the rising edge set to Double.POSITIVE_INFINITY to indicate no animation
         * @param offTime how long it should take for the color to turn off during the falling edge
         * @param isCycleColors overrides all other values and makes the state R A I N B O W
         * @param breathing smooths out the on and off phases with breathing
         */
        private State(double r, double g, double b, double onTime, double offTime, boolean breathing){
            colors.add(new Color(r/255, g/255, b/255));
            this.onTime = onTime;
            this.offTime = offTime;
            this.breathing = breathing;
        }

        private State() {
            isCycleColors = true;
            this.onTime = 100;
            this.offTime = 0;
            this.breathing = false;
        }

        private State(double onTime, double offTime, boolean breathing, Color... colors) {
            this.onTime = onTime;
            this.offTime = offTime;
            this.breathing = breathing;
            for (Color color : colors) {
                this.colors.add(color);
            }

        }
    }

    public LEDs() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.disableWhenLOS = true;
        config.stripType = LEDStripType.RGBW; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        mCandle.configAllSettings(config);

    }

    public void updateState() {
        setState(State.IDLE);

        if (mSuperstructure.hasEmergency) {
            setState(State.EMERGENCY);
            return;
        }

        if (mIndexer.getBallCount() == 1) {
            setState(State.ONE_BALL);
        }
        
        if (mIndexer.getBallCount() == 2) {
            setState(State.TWO_BALL);
        }

        if (mSuperstructure.isSpunUp()) {
            setState(State.SHOT_READY);
        }
    }

    public void updateLights() {

        
        
        if (mState.onTime == Double.POSITIVE_INFINITY) {
            // Positive indicates no animaiton
            Color color = mState.colors.get(0);
            mCandle.setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255));
            mCandle.configBrightnessScalar(1);
            return;
        }

        double timestamp = Timer.getFPGATimestamp();
        double lastTimestamp = mLastTimestamp;

        // The reason we need a delta time is ignore performance drops without the need for a second thread
        double deltaTime = timestamp - lastTimestamp;
        mLastTimestamp = timestamp;

        // Basic oscillation between 0 and onTime then 0 offTime
        if (mRising) {
            mPhase = mPhase + deltaTime;
            if (mPhase > mState.onTime) {
                mRising = false;
                mPhase = 0;
            }
        } else {
            mPhase = mPhase + deltaTime;
            if (mPhase > mState.offTime) {
                mRising = true;
                mPhase = 0;
                if (mColorPhase == (mState.colors.size() - 1)) {
                    mColorPhase = 0;
                } else {
                    mColorPhase++;
                }
            }
        }

        // Calculate the proper brightness values
        double percentBrightness = 0;
        if (mState.breathing) {
            if (mRising) {
                percentBrightness = (mPhase / mState.onTime);
            } else {
                percentBrightness = 1 - (mPhase / mState.offTime);
            }
        } else {
            percentBrightness = mRising ? 1 : 0;
        }


        int red = 255;
        int blue = 25;
        int green = 0;

        // Calculate R A I N B O W
        if (mState.isCycleColors) {
            red   = (int)(Math.sin(2*timestamp + 2) * 127 + 128);
            green = (int)(Math.sin(2*timestamp + 0) * 127 + 128);
            blue  = (int)(Math.sin(2*timestamp + 4) * 127 + 128);
        } else {
            Color color = mState.colors.get(mColorPhase);
            red = (int)(color.red * 255);
            green = (int)(color.green * 255);
            blue = (int)(color.blue * 255);

        }

        mCandle.setLEDs(red, green, blue);
        mCandle.configBrightnessScalar(percentBrightness);

    }

    public State getState() {
        return mState;
    }

    public void setState(State state) {
        mColorPhase = 0;
        mState = state;
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
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

                setState(State.RAINBOW);
            }

            @Override
            public void onLoop(double timestamp) {
                writePeriodicOutputs();
                updateState();
            }

            @Override
            public void onStop(double timestamp) {
                setState(State.DISABLED);
            }
        });
    }

}