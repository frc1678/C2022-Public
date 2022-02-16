package com.team1678.frc2022.auto.actions;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionAlignAction implements Action {

    // required subsystem instances
    Swerve mSwerve = Swerve.getInstance();
    Limelight mLimelight = Limelight.getInstance();

    // vision align controller
    ProfiledPIDController visionPIDController = new ProfiledPIDController(Constants.VisionAlignConstants.kP,
                                                        Constants.VisionAlignConstants.kI,
                                                        Constants.VisionAlignConstants.kD,
                                                        Constants.VisionAlignConstants.kThetaControllerConstraints);

    public VisionAlignAction() {
        visionPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void start() {
        
    }

    @Override
    public void update() {
        double rotation = 0.0;

        if (mLimelight.hasTarget()) {
            double currentAngle = mSwerve.getPose().getRotation().getRadians();
            double targetOffset = Math.toRadians(mLimelight.getOffset()[0]);
            rotation = visionPIDController.calculate(currentAngle, currentAngle - targetOffset);
        }

        mSwerve.drive(new Translation2d(), rotation, true, true);
    }

    @Override
    public boolean isFinished() {
        return mLimelight.isAimed();
    }

    @Override
    public void done() {}
}
