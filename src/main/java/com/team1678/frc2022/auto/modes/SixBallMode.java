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

public class SixBallMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // required PathWeaver file paths
    String file_path_a = "paths/SixBallPaths/6 Ball A.path";
    String file_path_b = "paths/SixBallPaths/6 Ball B.path";
    String file_path_c = "paths/SixBallPaths/6 Ball C.path";
    String file_path_d = "paths/SixBallPaths/6 Ball D.path";
    String file_path_e = "paths/SixBallPaths/6 Ball E.path";
    
	// trajectory actions
	SwerveTrajectoryAction driveToIntakeSecondCargo;
    SwerveTrajectoryAction driveToTerminalIntakePose;
    SwerveTrajectoryAction driveToSecondShotPose;
	SwerveTrajectoryAction driveToLastCargoIntake;
	SwerveTrajectoryAction driveToThirdShotPose;

    public SixBallMode() {

        SmartDashboard.putBoolean("Auto Finished", false);


        // define theta controller for robot heading
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.zeroToDefaultSpeedConfig);
        driveToIntakeSecondCargo = new SwerveTrajectoryAction(traj_path_a,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(135.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.constantSpeedConfig);
        driveToTerminalIntakePose = new SwerveTrajectoryAction(traj_path_b,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(225.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.constantSpeedConfig);
        driveToSecondShotPose = new SwerveTrajectoryAction(traj_path_c,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(200.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
                                                        
        Trajectory traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d, Constants.AutoConstants.constantSpeedConfig);
        driveToLastCargoIntake = new SwerveTrajectoryAction(traj_path_d,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(180.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e, Constants.AutoConstants.defaultToZeroSpeedConfig);
        driveToThirdShotPose = new SwerveTrajectoryAction(traj_path_e,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(270.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);                   
    
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running five ball mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // start spinning up for shot
        runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(driveToIntakeSecondCargo.getInitialPose().getX(), driveToIntakeSecondCargo.getInitialPose().getY(), Rotation2d.fromDegrees(135)))));

        // start intaking
        runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

        // start vision aiming to align drivetrain to target
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        
        // drive to intake second cargo
        runAction(driveToIntakeSecondCargo);

        // wait for 0.5 seconds before shooting
        runAction(new WaitAction(1.0));
        // shoot first two cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.25));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // stop vision aiming to control robot heading
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        // drive to pick up next two cargo from terminal
        runAction(driveToTerminalIntakePose);
        
        // wait for 1 second to finish intaking
        runAction(new WaitAction(1.0));

        // start vision aiming to align drivetrain to target
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        
        // drive to shoot at second shot pose
        runAction(driveToThirdShotPose);

        // wait for 0.5 seconds before shooting
        runAction(new WaitAction(0.5));
        // shoot second batch of cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.25));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));
        
        // stop vision aiming to control robot heading
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        // drive to intake last batch of cargo
        runAction(driveToLastCargoIntake);

        // start vision aiming to align to target for second shot
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

        // drive to third shot pose
        runAction(driveToThirdShotPose);

        // wait for 0.5 seconds before shooting
        runAction(new WaitAction(1.0));
        // shoot second batch of cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.25));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));
        
        // start vision aiming to align to target for second shot
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        // zero gyro properly
        // runAction(new LambdaAction(() -> mSwerve.zeroGyro(mSwerve.getYaw().getDegrees() - 270)));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);

    }

    @Override
    public Pose2d getStartingPose() {
        return driveToIntakeSecondCargo.getInitialPose();
    }
}
