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
import edu.wpi.first.wpilibj.util.Color;

public class LEDs extends Subsystem {

    public CANdle mCandle = new CANdle(Ports.CANDLE);
    
    public State mBottomState = State.RAINBOW;
    public State mTopState = State.RAINBOW;
    public static LEDs mInstance;
    public double mLastTimestamp = Timer.getFPGATimestamp();

    public static boolean alt = false;
    

    public final Superstructure mSuperstructure = Superstructure.getInstance();
    public final Indexer mIndexer = Indexer.getInstance();

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    public enum State {

        // Robot status indicators
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false), // LEDs OFF
        DISABLED(255, 20, 30, 9.0, 4.0, true), // Pink Breathing
        IDLE(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // Blue
        EMERGENCY(255, 0, 0, 0.3, 0.3, false), // Red Blinking

        // Test Colors
        RED(255, 0, 0, Double.POSITIVE_INFINITY, 0.0, false), // RED
        GREEN(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // GREEN
        BLUE(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // BLUE
        WHITE(255, 255, 255, 10, 5, false), // WHITE BLINKING
        MAGENTA(255, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // MAGENTA

        // Strip States
        ONE_BALL(0.2, 0.0, false, new Color(0.5, 0.1, 0.1), new Color(0.1, 0.1, 0.5)), // Alternating Yellow Blue
        TWO_BALL(Double.POSITIVE_INFINITY, 0.05, false, new Color(0.5, 0.1, 0.1)), // Fast Alternating Yellow Red

        // Regular States
        TARGET_VISIBLE(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // Slow Yellow
        SHOT_READY(255, 255, 0, 0.05, 0.05, false), // FAST Yellow

        // Fun Colors!
        MERICA(0.3, 0, false, new Color(1, 1, 1), new Color(1, 0, 0), new Color(0, 0, 1)), // Unused
        CITRUS(0.5, 0.5, true, new Color(0.23, 0.83, 0.18), new Color(0, 1, 0)), // Unused
        WORSE_RAINBOW(5, 0.5, true, new Color(1, 0, 0), new Color(1, 0.5, 0), new Color(1, 1, 0), new Color(0, 1, 0), new Color(0, 0, 1), new Color(0.29, 0, 0.51), new Color(0.58, 0, 0.83)), // Unused
        POLICE_RED(0.15, 0.0, true, new Color(1, 0, 0), new Color(0, 0, 0)),
        POLICE_BLUE(0.15, 0.0, true, new Color(0, 0, 0), new Color(0, 0, 1)),
        RAINBOW(); // :)

        List<Color> colors = new ArrayList<Color>();
        double onTime, offTime;
        boolean breathing = false;
        boolean isCycleColors = false;
        public int colorPhase = 0;
        public double phase = 0;
        public boolean rising = false;

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
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1.0; // dim the LEDs to half brightness
        mCandle.configAllSettings(config);

    }

    public double getCurrentDraw() {
        return mCandle.getCurrent();
    }

    public double getTemperature() {
        return mCandle.getTemperature();
    }

    public void updateState() {
        if (mSuperstructure.hasEmergency) {
            setBottomState(State.EMERGENCY);
            setTopState(State.EMERGENCY);
            return;
        }

        if (mIndexer.getBallCount() == 1) {
            setTopState(State.ONE_BALL);
        } else if (mIndexer.getBallCount() == 2) {
            setTopState(State.TWO_BALL);
        } else {
            setTopState(State.IDLE);
        }

        if (Limelight.getInstance().hasTarget()) {
            setBottomState(State.TARGET_VISIBLE);
        } else if (mSuperstructure.isSpunUp()) {
            setBottomState(State.SHOT_READY);
        } else {
            setBottomState(State.IDLE);
        }
    }

    public void updateLights() {

        double timestamp = Timer.getFPGATimestamp();
        double lastTimestamp = mLastTimestamp;

        // The reason we need a delta time is ignore performance drops without the need for a second thread
        double deltaTime = timestamp - lastTimestamp;
        mLastTimestamp = timestamp;

        // For some reason we need to alternate setting states
        if (alt) {
            updateLEDsFromState(mBottomState, 0, 20, deltaTime * 2, timestamp);
        } else {
            updateLEDsFromState(mTopState, 20, 30, deltaTime * 2, timestamp);
        }
        alt = !alt;

    }

    public void updateLEDsFromState(State state, int start, int end, double deltaTime, double timestamp) {
        
        if (state.onTime == Double.POSITIVE_INFINITY) {
            // Positive indicates no animaiton
            Color color = state.colors.get(0);
            mCandle.setLEDs((int)(color.red * 255), (int)(color.green * 255), (int)(color.blue * 255), 0, start, end);
            return;
        }

        // Basic oscillation between 0 and onTime then 0 offTime
        if (state.rising) {
            state.phase = state.phase + deltaTime;
            if (state.phase > state.onTime) {
                state.rising = false;
                state.phase = 0;
            }
        } else {
            state.phase = state.phase + deltaTime;
            if (state.phase > state.offTime) {
                state.rising = true;
                state.phase = 0;
                if (state.colorPhase == (state.colors.size() - 1)) {
                    state.colorPhase = 0;
                } else {
                    state.colorPhase++;
                }
            }
        }

        // Calculate the proper brightness values
        double percentBrightness = 0;
        if (state.breathing) {
            if (state.rising) {
                percentBrightness = (state.phase / state.onTime);
            } else {
                percentBrightness = 1 - (state.phase / state.offTime);
            }
        } else {
            percentBrightness = state.rising ? 1 : 0;
        }


        int red = 255;
        int blue = 25;
        int green = 0;

        // Calculate R A I N B O W
        if (state.isCycleColors) {
            red   = (int)(Math.sin(5*timestamp + 2) * 127 + 128);
            green = (int)(Math.sin(5*timestamp + 0) * 127 + 128);
            blue  = (int)(Math.sin(5*timestamp + 4) * 127 + 128);
        } else {
            Color color = state.colors.get(state.colorPhase);
            red = (int)(color.red * percentBrightness * 255);
            green = (int)(color.green * percentBrightness * 255);
            blue = (int)(color.blue * percentBrightness * 255);

        }

        mCandle.setLEDs(red, green, blue, 0, start, end);

    }

    public State getState() {
        return mBottomState;
    }

    public State getStripState() {
        return mTopState;
    }

    public void setBottomState(State state) {
        state.colorPhase = 0;
        mBottomState = state;
    }

    public void setTopState(State state) {
        state.colorPhase = 0;
        mTopState = state;
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
            }

            @Override
            public void onLoop(double timestamp) {
                updateState();
            }

            @Override
            public void onStop(double timestamp) {
                setBottomState(State.DISABLED);
                setTopState(State.DISABLED);
            }
        });
    }

}