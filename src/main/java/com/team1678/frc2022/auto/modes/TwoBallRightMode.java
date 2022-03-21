package com.team1678.frc2022.auto.modes;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.auto.actions.WaitAction;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TwoBallRightMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // required PathWeaver file paths
    String file_path_a = "paths/TwoBallPaths/2 Ball Right A.path";
    String file_path_b = "paths/TwoBallPaths/2 Ball Right B.path";
    
	// trajectory actions
	SwerveTrajectoryAction driveToIntakeCargo;
    SwerveTrajectoryAction driveToShotPose;

    public TwoBallRightMode() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.zeroToDefaultSpeedConfig);
        driveToIntakeCargo = new SwerveTrajectoryAction(traj_path_a,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(210.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.defaultToZeroSpeedConfig);
        driveToShotPose = new SwerveTrajectoryAction(traj_path_b,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(225.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running two ball right mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(driveToIntakeCargo.getInitialPose().getX(),
                                                                          driveToIntakeCargo.getInitialPose().getY(),
                                                                          Rotation2d.fromDegrees(215)))));

        // start spinning up for shot
        runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));
        // start intaking
        runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

        // start vision aiming to align drivetrain to target
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        
        // run trajectory to intake second cargo
        runAction(driveToIntakeCargo);
        
        // wait for 0.5 seconds to finish intaking cargo
        runAction(new WaitAction(0.5));
        
        // run trajectory to shot pose
        runAction(driveToShotPose);

        // wait 1.0 seconds
        runAction(new WaitAction(1.0));

        // shoot 
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(5.0));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // runAction(new LambdaAction(() -> mSwerve.zeroGyro(driveToShotPose.)));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);

    }

    @Override
    public Pose2d getStartingPose() {
        return driveToIntakeCargo.getInitialPose();
    }
}
