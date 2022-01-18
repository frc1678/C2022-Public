package com.team1678.frc2022.auto.modes;

import java.io.IOException;
import java.util.List;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.ParallelAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.auto.actions.VisionAlignAction;
import com.team1678.frc2022.auto.actions.WaitAction;
import com.team1678.frc2022.subsystems.Intake;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class FiveBallMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve mSwerve = Swerve.getInstance();
    private final Intake mIntake = Intake.getInstance();
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
                                                            () -> Rotation2d.fromDegrees(20.0),
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
                                                            () -> Rotation2d.fromDegrees(20.0),
                                                            mSwerve::getWantAutoVisionAim,
                                                            mSwerve::setModuleStates);
        
                                                        
    
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running five ball mode auto!");

        runAction(new LambdaAction(() -> mSuperstructure.setWantSpinUp(true)));
        
        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(driveToIntakeFirstCargo.getInitialPose().getX(), driveToIntakeFirstCargo.getInitialPose().getY(), Rotation2d.fromDegrees(90)))));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(2.0));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // start intaking
        runAction(new LambdaAction(() -> mIntake.setState(Intake.State.INTAKING)));

        runAction(driveToIntakeFirstCargo);
        runAction(driveToIntakeSecondCargo);

        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        runAction(driveToFirstShot);
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new ParallelAction(List.of(new WaitAction(2.0), new VisionAlignAction(), new LambdaAction(() -> mSuperstructure.setWantShoot(false)))));

        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        runAction(driveToIntakeAtTerminal);
        runAction(new WaitAction(2.0));

        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));
        runAction(driveToShootFromTerminal);
        runAction(new VisionAlignAction());
        new LambdaAction(() -> mSuperstructure.setWantShoot(true));
        runAction(new WaitAction(2.0));
        new LambdaAction(() -> mSuperstructure.setWantShoot(false));
        new LambdaAction(() -> mSuperstructure.setWantSpinUp(false));
        
        System.out.println("Finished auto!");
    }
}
