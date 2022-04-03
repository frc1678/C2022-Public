package com.team1678.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.Ports;
import com.team1678.frc2022.loops.ILooper;
import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.subsystems.ColorSensor.ColorChoices;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends Subsystem {

    private final CANdle mCandle = new CANdle(Ports.CANDLE, "canivore1");

    private static LEDs mInstance;

    public static LEDs getInstance() {
        if (mInstance == null) {
            mInstance = new LEDs();
        }
        return mInstance;
    }

    private double timestamp = 0.0;

    private final boolean mUseSmartdash = false; // if we want to manual control lights using shuffleboard
    private boolean mLastUpdatedTop = false; // alternate updating sections to avoid overrunning the CANdle

    // led sections
    private LEDStatus mTopStatus = new LEDStatus(14, 28);  
    private LEDStatus mLeftBottomStatus = new LEDStatus(28, 40);
    private LEDStatus mRightBottomStatus = new LEDStatus(0, 14);

    // shuffleboard selectors
    private SendableChooser<State> mTopStateChooser;
    private SendableChooser<State> mBottomStateChooser;

    // animation to run when disabled
    private Animation mRedAllianceAnimation = new ColorFlowAnimation(255, 0, 0, 0, 0.5, 40, Direction.Forward);
    private Animation mBlueAllianceAnimation = new ColorFlowAnimation(0, 0, 255, 0, 0.5, 40, Direction.Backward);
    private Animation mNoAllianceAnimation =  new ColorFlowAnimation(185, 64, 255, 0, 0.5, 40, Direction.Backward);


    // led states
    public enum State{
        OFF("OFF", Double.POSITIVE_INFINITY, Color.off()),
        EMERGENCY("EMERGENCY", 0.05, new Color(255, 0, 0), Color.off()),

        SOLID_RED("SOLID_RED", Double.POSITIVE_INFINITY, new Color(255, 0, 0)),
        SOLID_PINK("SOLID_PINK", Double.POSITIVE_INFINITY, new Color(255, 18, 143)),
        SOLID_GREEN("SOLID_GREEN", Double.POSITIVE_INFINITY, new Color(0, 255, 8)),
        SOLID_PURPLE("SOLID_PURPLE", Double.POSITIVE_INFINITY, new Color(196, 18, 255)),
        SOLID_ORANGE("SOLID_ORANGE", Double.POSITIVE_INFINITY, new Color(255, 53, 13)),
        SOLID_YELLOW("SOLID_YELLOW", Double.POSITIVE_INFINITY, new Color(243, 255, 18)),
        SOLID_CYAN("SOLID_CYAN", Double.POSITIVE_INFINITY, new Color(18, 239, 255)),

        FLASHING_RED("FLASHING_RED", 0.05, new Color(255, 0, 0), Color.off()),
        FLASHING_PINK("FLASHING_PINK", 0.05, new Color(255, 20, 0), Color.off()),
        FLASHING_GREEN("FLASHING_GREEN", 0.05, new Color(0, 255, 0), Color.off()),
        FLASHING_PURPLE("FLASHING_PURPLE", 0.05, new Color(255, 0, 255), Color.off()),
        FLASHING_ORANGE("FLASHING_ORANGE", 0.05, new Color(255, 53, 0), Color.off()),
        FLASHING_YELLOW("FLASHING_YELLOW", 0.05, new Color(255, 255, 0), Color.off()),
        FLASHING_CYAN("FLASHING_CYAN", 0.05, new Color(0, 255, 255), Color.off());

        Color[] colors; // array of colors to iterate over
        double interval; // time in seconds between states
        String name; // name of state

        private State(String name, double interval, Color... colors) {
            this.colors = colors;
            this.interval = interval;
            this.name = name;
        }

        public String getName() {
            return name;
        }
    }

    public LEDs() {
        configureCandle(); // set CTRE configurations for CANdle

        // create sendable choosers for shuffleboard
        if (mUseSmartdash) {
            mTopStateChooser = new SendableChooser<>();
            mBottomStateChooser = new SendableChooser<>();
            for (State state : State.values()) {
                mTopStateChooser.addOption(state.getName(),  state);
                mBottomStateChooser.addOption(state.getName(),  state);
            }
            mTopStateChooser.setDefaultOption("OFF", State.OFF);
            mBottomStateChooser.setDefaultOption("OFF", State.OFF);
            SmartDashboard.putData("Top LEDs", mTopStateChooser);
            SmartDashboard.putData("Bottom LEDs", mBottomStateChooser);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mCandle.clearAnimation(0);
                applyStates(State.OFF, State.OFF); 
            }

            @Override
            public void onLoop(double timestamp) {
                
            }

            @Override
            public void onStop(double timestamp) {
                mTopStatus.reset();
                mLeftBottomStatus.reset();
                mRightBottomStatus.reset();
            }
        });
    }

    @Override
    public void readPeriodicInputs() {
        outputTelemtry();
        timestamp = Timer.getFPGATimestamp(); // update timestamp for color cycling
        if (mUseSmartdash) { // pull states from smartdash
            applyStates(mTopStateChooser.getSelected(), mBottomStateChooser.getSelected()); 
        }
    }

    public void updateState() { /// write state to candle
        // alternate updating each section of leds to avoid overrunning the candle
        if (mLastUpdatedTop) {
            updateBottomLeds();
            mLastUpdatedTop = false;
        } else {
            updateTopLeds();
            mLastUpdatedTop = true;
        }
    }

    private void updateBottomLeds() {
        // check if we need to cycle to next color
        if (mLeftBottomStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mLeftBottomStatus.lastSwitchTime >= mLeftBottomStatus.state.interval) {
                mLeftBottomStatus.nextColor();
                mLeftBottomStatus.lastSwitchTime = timestamp;
            }
        }

        Color bottomColor = mLeftBottomStatus.getWantedColor();

        mCandle.setLEDs(bottomColor.r, bottomColor.g, bottomColor.b, 0, mLeftBottomStatus.startIDx,
                mLeftBottomStatus.LEDCount);

        mCandle.setLEDs(bottomColor.r,
                bottomColor.g, bottomColor.b, 0, mRightBottomStatus.startIDx, mRightBottomStatus.LEDCount);
    
    }

    private void updateTopLeds() {
        // check if we need to cycle to next color
        if (mTopStatus.state.interval != Double.POSITIVE_INFINITY) {
            if (timestamp - mTopStatus.lastSwitchTime >= mTopStatus.state.interval) {
                mTopStatus.nextColor();
                mTopStatus.lastSwitchTime = timestamp;
            }
        }

        Color topColor = mTopStatus.getWantedColor();

        mCandle.setLEDs(topColor.r, topColor.g, topColor.b, 0, mTopStatus.startIDx, mTopStatus.LEDCount);
    }

    // setter functions
    public void applyStates(State topState, State bottomState) {
        mTopStatus.setState(topState);
        mLeftBottomStatus.setState(bottomState);
    }

    public void applyTopState(State state) {
        mTopStatus.setState(state);
    }

    public void applyBottomState(State state) {
        mLeftBottomStatus.setState(state);
    }

    // apply configuration to candle
    private void configureCandle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1.0;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        mCandle.configAllSettings(configAll, Constants.kLongCANTimeoutMs);
        mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
        mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
    }

    public void updateColor(ColorChoices allianceColor) {
        if (allianceColor.equals(ColorChoices.RED)) {
            mCandle.animate(mRedAllianceAnimation);    
        } else if (allianceColor.equals(ColorChoices.BLUE)) {
            mCandle.animate(mBlueAllianceAnimation); 
        } else {
            mCandle.animate(mNoAllianceAnimation); 
        }
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
    }

    @Override
    public boolean checkSystem() {
        return false;
    }
    
    // getter functions
    public State getTopState() {
        return mTopStatus.state;
    }

    public State getBottomState() {
        return mLeftBottomStatus.state;
    }

    public boolean getUsingSmartdash() {
        return mUseSmartdash;
    }

    private void outputTelemtry() {
        SmartDashboard.putString("Top LED Status", getTopState().name);
        SmartDashboard.putString("Bottom LED Status", getBottomState().name);

        SmartDashboard.putString("Top LED Colors", mTopStatus.getWantedColor().toString());
        SmartDashboard.putString("Bottom LED Colors", mLeftBottomStatus.getWantedColor().toString());        
    }

    // class for holding information about each section
    private class LEDStatus {
        private State state = State.OFF; // current state
        private double lastSwitchTime = 0.0; // timestampe of last color cycle
        private int colorIndex = 0; // tracks current color in array
        private int startIDx, LEDCount; // start and end of section

        public LEDStatus(int startIndex, int endIndex) {
            startIDx = startIndex;
            LEDCount = endIndex - startIndex;
        }

        public void setState(State wantedState) {
            if (wantedState != state) {
                colorIndex = 0;
                lastSwitchTime = Timer.getFPGATimestamp();
                state = wantedState;
            }
        }

        public Color getWantedColor() {
            Color color;
            try {
                color = state.colors[colorIndex];
            } catch (Exception e) {
                color = Color.off();
            }
            return color;
        }

        // cycle to next color in array
        public void nextColor() { 
            if (state.colors.length == 1) {
                return;
            }
            if (colorIndex == state.colors.length - 1) {
                colorIndex = 0;
            } else {
                colorIndex++;
            }
        }

        public void reset() {
            state = State.OFF;
            lastSwitchTime = 0.0;
            colorIndex = 0;
        }
    }

    // class to hold rgb values for a color
    private static class Color {
        public int r;
        public int g;
        public int b;

        public Color(int red, int green, int blue) {
            r = red;
            g = green;
            b = blue;
        }

        public static Color off() {
            return new Color(0, 0, 0);
        }

        @Override
        public String toString() {
            return "(" + r + "," + g + "," + b + ")";
        }
    }
}
