package com.team1678.frc2022.auto.modes;

import java.io.IOException;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.auto.actions.WaitAction;
import com.team1678.frc2022.subsystems.Intake;
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
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Intake mIntake = Intake.getInstance();

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
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(225.0),
                                                            s_Swerve::setModuleStates);

        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.constantSpeedConfig);
        driveToIntakeSecondCargo = new SwerveTrajectoryAction(traj_path_b,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(120.0),
                                                            s_Swerve::setModuleStates);

        Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.constantSpeedConfig);
        driveToFirstShot = new SwerveTrajectoryAction(traj_path_c,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(20.0),
                                                            s_Swerve::setModuleStates);
                                                        
        Trajectory traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d, Constants.AutoConstants.constantSpeedConfig);
        driveToIntakeAtTerminal = new SwerveTrajectoryAction(traj_path_d,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(235.0),
                                                            s_Swerve::setModuleStates);

        Trajectory traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e, Constants.AutoConstants.defaultToZeroSpeedConfig);
        driveToShootFromTerminal = new SwerveTrajectoryAction(traj_path_e,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(20.0),
                                                            s_Swerve::setModuleStates);
        
                                                        
    
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running five ball mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(new Pose2d(driveToIntakeFirstCargo.getInitialPose().getX(), driveToIntakeFirstCargo.getInitialPose().getY(), Rotation2d.fromDegrees(90)))));

        // start intaking
        runAction(new LambdaAction(() -> mIntake.setState(Intake.State.INTAKING)));

        runAction(driveToIntakeFirstCargo);
        runAction(driveToIntakeSecondCargo);
        runAction(driveToFirstShot);
        runAction(new WaitAction(2.0));
        runAction(driveToIntakeAtTerminal);
        runAction(new WaitAction(2.0));
        runAction(driveToShootFromTerminal);
        
        System.out.println("Finished auto!");
    }
}
