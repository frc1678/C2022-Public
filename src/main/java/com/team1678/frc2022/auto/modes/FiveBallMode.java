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

public class FiveBallMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve mSwerve = Swerve.getInstance();
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    // required PathWeaver file paths
    String file_path_a = "paths/FiveBallPaths/5 Ball A.path";
    String file_path_b = "paths/FiveBallPaths/5 Ball B.path";
    String file_path_c = "paths/FiveBallPaths/5 Ball C.path";
    String file_path_d = "paths/FiveBallPaths/5 Ball D.path";
    String file_path_e = "paths/FiveBallPaths/5 Ball E.path";
    
	// trajectory actions
	SwerveTrajectoryAction driveToIntakeFirstCargo;
    SwerveTrajectoryAction driveToIntakeSecondCargo;
    SwerveTrajectoryAction driveToFirstShot;
	SwerveTrajectoryAction driveToIntakeAtTerminal;
	SwerveTrajectoryAction driveToShootFromTerminal;

    public FiveBallMode() {

        SmartDashboard.putBoolean("Auto Finished", false);


        // define theta controller for robot heading
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.zeroToDefaultSpeedConfig);
        driveToIntakeFirstCargo = new SwerveTrajectoryAction(traj_path_a,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(225.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.constantSpeedConfig);
        driveToIntakeSecondCargo = new SwerveTrajectoryAction(traj_path_b,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(125.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.constantSpeedConfig);
        driveToFirstShot = new SwerveTrajectoryAction(traj_path_c,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(200.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
                                                        
        Trajectory traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d, Constants.AutoConstants.constantSpeedConfig);
        driveToIntakeAtTerminal = new SwerveTrajectoryAction(traj_path_d,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(235.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);

        Trajectory traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e, Constants.AutoConstants.defaultToZeroSpeedConfig);
        driveToShootFromTerminal = new SwerveTrajectoryAction(traj_path_e,
                                                            mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(200.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
        
                                                        
    
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running five ball mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(
            driveToIntakeFirstCargo.getInitialPose().getX(),
            driveToIntakeFirstCargo.getInitialPose().getY(),
            Rotation2d.fromDegrees(270)))));
        
        // start spinning up for shot
        runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));

        // wait 1 second for curr calibration on hood to complete
        runAction(new WaitAction(1.0));

        // shoot first cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.0));
        
        // start intaking
        runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

        // stop shooting
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // run trajectories for first and second cargo intakes
        runAction(driveToIntakeFirstCargo);
        runAction(driveToIntakeSecondCargo);

        // start vision aiming to align drivetrain to target
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        
        // run trajectory to drive to first shot pose
        runAction(driveToFirstShot);

        // wait for 0.5 seconds before the shot
        runAction(new WaitAction(0.5));

        // shoot second and third cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.0));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // stop vision aiming to control robot heading
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        // run trajectory to drive to intake at terminal
        runAction(driveToIntakeAtTerminal);
        runAction(new WaitAction(0.5));

        // start vision aiming to align to target for second shot
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

        // run trajectory to drive to second shot pose
        runAction(driveToShootFromTerminal);
        
        // shoot fourth and fifth cargo 
        runAction(new WaitAction(0.5));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);

    }
}
