package com.team1678.frc2022.subsystems;

import com.team1678.frc2022.loops.Loop;
import com.team1678.frc2022.regressions.ShooterRegression;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.controlboard.ControlBoard;
import com.team1678.frc2022.controlboard.CustomXboxController.Side;
import com.team1678.frc2022.loops.ILooper;

public class Superstructure extends Subsystem {

    /* Superstructure Instance */
    private static Superstructure mInstance;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    };

    /* REQUIRED INSTANCES */
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Indexer mIndexer = Indexer.getInstance();
    private final Shooter mShooter = Shooter.getInstance();
    private final Hood mHood = Hood.getInstance();
    private final Limelight mLimelight = Limelight.getInstance();

    /* SUPERSTRUCTURE ACTIONS */
    public boolean NONE = false;
    public boolean INTAKE = false;
    public boolean OUTTAKE = false;
    public boolean PREP = false;
    public boolean SHOOT = false;
    public boolean FENDOR = false;

    /* SETPOINT VARIABLES */
    public double mShooterSetpoint = 0.0;
    public double mHoodSetpoint = 10.0; // TODO: arbitrary value, change

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                updateOperatorCommands();
                maybeUpdateGoalFromVision(timestamp);
                setGoals();
                outputTelemetry();
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    public boolean isSpunUp() {
        return mShooter.spunUp();
    }

    /* hard set superstructure actions outside operator input */
    public void setWantIntake(boolean intake) {
        if (INTAKE != intake) {
            INTAKE = intake;
        }
    }
    public void setWantOuttake(boolean outtake) {
        if (OUTTAKE != outtake) {
            OUTTAKE = outtake;
        }
    }
    public void setWantPrep(boolean wants_prep) {
        PREP = wants_prep;
    }
    public void setWantShoot(boolean wants_shoot) {
        SHOOT = wants_shoot;
    }
    public void setShooterVelocity(double velocity) {
        mShooterSetpoint = velocity;
    }
    public void setWantIntake() {
        setWantIntake(!INTAKE);
    }

    /* UPDATE OPERATOR COMMANDS TO CALL SUPERSTRUCTURE ACTIONS */
    public void updateOperatorCommands() {
        // control intake vs. outtake actions
        if (mControlBoard.operator.getTrigger(Side.RIGHT)) {
            INTAKE = true;
        } else if (mControlBoard.operator.getTrigger(Side.LEFT)) {
            OUTTAKE = true;
        } else {
            INTAKE = false;
            OUTTAKE = false;
        }

        // control shooting
        if (mControlBoard.operator.getController().getYButtonPressed()) {
            SHOOT = !SHOOT;
        }

        // control prepping
        if (mControlBoard.operator.getController().getAButtonPressed()) {
            PREP = !PREP;
        }
    }

    /* UPDATE SHOOTER AND HOOD GOAL WHEN VISION AIMING */
    public synchronized void maybeUpdateGoalFromVision(double timestamp) {
        if (mLimelight.hasTarget()) {
            Optional<Double> distance_to_target = mLimelight.getDistanceToTarget();
            if (distance_to_target.isPresent()) {
                mShooterSetpoint = getShooterSetpointFromRegression(distance_to_target.get());
                mHoodSetpoint = getHoodSetpointFromRegression(distance_to_target.get());
            }
        }
    }

    /* UPDATE SUBSYSTEM STATES + SETPOINTS AND SET GOALS */
    public void setGoals() {
        // subsystem wanted actions and setpoints to be set based on superstructure actions
        Intake.WantedAction real_intake;
        Indexer.WantedAction real_indexer;
        double real_shooter;
        double real_hood;

        /* UPDATE SUBSYSTEM WANTED ACTIONS AS REQUESTED */

        // update hood setpoint
        real_hood = mHoodSetpoint;

        // update shooter setpoint
        if (PREP) {
            real_shooter = mShooterSetpoint;
        } else {
            real_shooter = 0.0;
        }

        // intake and indexer actions
        if (SHOOT) {
            real_intake = Intake.WantedAction.NONE;

            if (isSpunUp()) {
                real_indexer = Indexer.WantedAction.FEED;
            } else {
                real_indexer = Indexer.WantedAction.INDEX;
            }
        } else {
            if (INTAKE) {
                real_intake = Intake.WantedAction.INTAKE;
                real_indexer = Indexer.WantedAction.INDEX;
            } else if (OUTTAKE) {
                real_intake = Intake.WantedAction.REVERSE;
                real_indexer = Indexer.WantedAction.REVERSE;
            } else {
                real_intake = Intake.WantedAction.NONE;
                real_indexer = Indexer.WantedAction.INDEX;
            }
        }

        /* SET SUBSYSTEM STATES/SETPOINTS BASED ON WANTED ACTIONS */
        
        // set intake and indexer states
        mIntake.setState(real_intake);
        mIndexer.setState(real_indexer);

        // set shooter subsystem setpoint
        if (Math.abs(real_shooter) < Util.kEpsilon) {
            mShooter.setOpenLoop(0); // open loop if rpm goal is 0, to smooth spin down and stop belt skipping
        } else {
            mShooter.setVelocity(mShooterSetpoint);
        }

        // set hood subsystem setpoint
        // clamp the hood goal between min and max hard stops for hood angle
        real_hood = Util.clamp(real_hood,
                Constants.HoodConstants.kHoodServoConstants.kMinUnitsLimit,
                Constants.HoodConstants.kHoodServoConstants.kMaxUnitsLimit); 
        mHood.setSetpointMotionMagic(real_hood);
    }

    /* GET SHOOTER AND HOOD SETPOINTS FROM SUPERSTRUCTURE CONSTANTS REGRESSION */
    private double getShooterSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Shooting RPM", 0);
        } else if (ShooterRegression.kUseFlywheelAutoAimPolynomial) {
            return ShooterRegression.kFlywheelAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }
    private double getHoodSetpointFromRegression(double range) {
        if (ShooterRegression.kUseSmartdashboard) {
            return SmartDashboard.getNumber("Hood Angle", 0);
        } else if (ShooterRegression.kUseHoodAutoAimPolynomial) {
            return ShooterRegression.kHoodAutoAimPolynomial.predict(range);
        } else {
            return ShooterRegression.kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range)).value;
        }
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    /* Output superstructure actions and other related statuses */
    public void outputTelemetry() {
        // superstructure action statuses
        SmartDashboard.putBoolean("Intaking", INTAKE);
        SmartDashboard.putBoolean("Outtaking", OUTTAKE);
        SmartDashboard.putBoolean("Prepping", PREP);
        SmartDashboard.putBoolean("Shooting", SHOOT);
        SmartDashboard.putBoolean("Fendor Shooting", FENDOR);

        // other status variables
        SmartDashboard.putBoolean("Is Spun Up", isSpunUp());
        SmartDashboard.putBoolean("Has Vision Target", mLimelight.hasTarget());
        SmartDashboard.putBoolean("Is Vision Aimed", mLimelight.isAimed());
    }
}
