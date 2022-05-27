package com.team1678.frc2022.controlboard;

import com.team1678.frc2022.Constants;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class CustomXboxController {
    private final XboxController mController;

    public enum Side {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    CustomXboxController(int port) {
        mController = new XboxController(port);
    }


    double getAxis(Side side, Axis axis) {
        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
    }

    public boolean getTrigger(Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3) > Constants.kTriggerThreshold;
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    public void setRumble(boolean on) {
        mController.setRumble(RumbleType.kRightRumble, on ? 1 : 0);
    }

    public XboxController getController() {
        return mController;
    }
}
