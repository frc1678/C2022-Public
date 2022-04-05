package com.team1678.frc2022.shuffleboard;

import java.util.Optional;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.RobotState;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;
import com.team254.lib.vision.AimingParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private Field2d mField2d;
    private Swerve mSwerve = Swerve.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private RobotState mRobotState = RobotState.getInstance();

    private Pose2d[] mModulePoses = new Pose2d[4];
    private Pose2d mRobotPose;

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void updateSwervePoses() {
        mRobotPose = mSwerve.getPose();
        for (int i = 0; i < mModulePoses.length; i++) {
            Translation2d updatedPosition = Constants.SwerveConstants.swerveModuleLocations[i]
                    .rotateBy(mRobotPose.getRotation()).plus(mRobotPose.getTranslation());
            mModulePoses[i] = new Pose2d(updatedPosition, mSwerve.getStates()[i].angle.plus(mRobotPose.getRotation()));
        }
    }

    public void update() {
        updateSwervePoses();

        mField2d.setRobotPose(mSwerve.getPose());
        mField2d.getObject("Swerve Modules").setPoses(mModulePoses);

        Optional<AimingParameters> target = mSuperstructure.getRealAimingParameters();
        if (target.isPresent()) {
            Pose2d robotToTarget = target.get().getVehicleToGoal().getWpilibPose2d();
            mField2d.getObject("Target")
                    .setPose(new Pose2d(mRobotPose.getTranslation().plus(robotToTarget.getTranslation()),
                            mRobotPose.getRotation().rotateBy(robotToTarget.getRotation())));
        }

        mField2d.getObject("Predicted Robot Pose").setPose(mRobotState.getPredictedFieldToVehicle(Constants.VisionConstants.kLookaheadTime).getWpilibPose2d());
    }
}
