package com.team1678.frc2022.controlboard;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.controlboard.CustomXboxController.Axis;
import com.team1678.frc2022.controlboard.CustomXboxController.Button;
import com.team1678.frc2022.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;

    private int mLastDpadLeft = -1;
    private int mLastDpadRight = -1;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static ControlBoard mInstance = null;

    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180),

        FAR_FENDER(143),
        RIGHT_FENDER(233),
        LEFT_FENDER(53),
        CLOSE_FENDER(323);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public static ControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new ControlBoard();
        }

        return mInstance;
    }

    private final CustomXboxController driver;
    public final CustomXboxController operator;

    private ControlBoard() {
        driver = new CustomXboxController(0);
        operator = new CustomXboxController(Constants.kButtonGamepadPort);
    }
    
    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getController().getRawAxis(1);
        double strafeAxis = driver.getController().getRawAxis(0);

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);


        forwardAxis = Constants.SwerveConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.SwerveConstants.invertXAxis ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.norm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.x(), tAxes.y(), true);
            Translation2d deadband_vector = Translation2d.fromPolar(deadband_direction, kSwerveDeadband);

            double scaled_x = tAxes.x() - (deadband_vector.x()) / (1 - deadband_vector.x());
            double scaled_y = tAxes.y() - (deadband_vector.y()) / (1 - deadband_vector.y());
            return new Translation2d(scaled_x, scaled_y).scale(Constants.SwerveConstants.maxSpeed);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X);
        rotAxis = Constants.SwerveConstants.invertRAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) {
            return 0.0;
        } else {
            return Constants.SwerveConstants.maxAngularVelocity * (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START) && driver.getButton(Button.BACK);
    }

    public SwerveCardinal getSwerveSnap() {

        // FENDER SNAPS
        if (driver.getButton(Button.A)) {
            return SwerveCardinal.CLOSE_FENDER;
        }
        if (driver.getButton(Button.B)) {
            return SwerveCardinal.LEFT_FENDER;
        }
        if (driver.getButton(Button.X)) {
            return SwerveCardinal.RIGHT_FENDER;
        }
        if (driver.getButton(Button.Y)) {
            return SwerveCardinal.FAR_FENDER;
        }

        // CARDINAL SNAPS

        switch (driver.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.LEFT;
            case kDpadRight:
                return SwerveCardinal.RIGHT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
            
        
    }

    public int getHoodManualAdjustment() {
        int pov_read = operator.getController().getPOV();
        switch(pov_read){
            case 0:
                return 1;
            case 180:
                return -1;
            default:
                return 0;
        }
    }

    // Align swerve drive with target
    public boolean getVisionAlign() {
        return driver.getButton(Button.RB);
    }

    //Locks wheels in X formation
    public boolean getBrake() {
        return driver.getButton(Button.LB);
    }

    public boolean getDisableColorLogic() {
        boolean wasPressed = operator.getController().getPOV() == kDpadRight && mLastDpadRight != kDpadRight;
        SmartDashboard.putNumber("DPAD", operator.getController().getPOV());
        SmartDashboard.putNumber("Last DPAD", mLastDpadRight);
        mLastDpadRight = operator.getController().getPOV();
        return wasPressed;
    }

    public boolean getDisableIntakeLogic() {
        boolean wasPressed = operator.getController().getPOV() == kDpadRight && mLastDpadLeft != kDpadLeft;
        mLastDpadLeft = operator.getController().getPOV();
        return wasPressed;
    }

    // Climber Controls
    public boolean getClimbMode() {
        return operator.getButton(Button.LB) && operator.getButton(Button.RB) && operator.getTrigger(Side.LEFT) && operator.getTrigger(Side.RIGHT);
    }

    public boolean getExitClimbMode() {
        return operator.getButton(Button.BACK) && operator.getButton(Button.START);
    }
    
    public boolean getTraversalClimb() {
        return operator.getButton(Button.LB) && operator.getController().getYButtonPressed();
    }
    
}

