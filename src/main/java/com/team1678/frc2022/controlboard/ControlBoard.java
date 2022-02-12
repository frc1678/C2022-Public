package com.team1678.frc2022.controlboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.controlboard.CustomXboxController.Axis;
import com.team1678.frc2022.controlboard.CustomXboxController.Button;
import com.team1678.frc2022.controlboard.CustomXboxController.Side;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.DelayedBoolean;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = Constants.stickDeadband;
    private final double kOperatorDeadband = 0.15;

    private int mDPadUp = -1;
    private int mDPadDown = -1;
    private int mDPadRight = -1;
    private int mDPadLeft = -1;

    private final double kDPadDelay = 0.02;
    private DelayedBoolean mDPadValid;

    private static ControlBoard mInstance = null;

    public enum SwerveCardinal {
        NONE(0),
        FRONT(0),
        LEFT(90),
        RIGHT(-90),
        BACk(180);

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
        if (driver.getButton(Button.A)) {
            return SwerveCardinal.BACk;
        }
        if (driver.getButton(Button.B)) {
            return SwerveCardinal.RIGHT;
        }
        if (driver.getButton(Button.X)) {
            return SwerveCardinal.LEFT;
        }
        if (driver.getButton(Button.Y)) {
            return SwerveCardinal.FRONT;
        }
        return SwerveCardinal.NONE;
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

    //Intake Controls
    public boolean getIntake() {
        return operator.getTrigger(Side.RIGHT);
    }

    public boolean getOuttake() {
        return operator.getTrigger(Side.LEFT);
    }

    public boolean getSpitting() {
        return operator.getController().getRightBumper();
    }

    //Indexer Controls
    public boolean getElevating() {
        return operator.getButton(Button.B);
    }

    public boolean getIndexing() {
        return operator.getButton(Button.Y);
    }

    public boolean getHopping() {
        return operator.getButton(Button.X);
    }

    public boolean getReversing() {
        return operator.getButton(Button.A);
    }
}

