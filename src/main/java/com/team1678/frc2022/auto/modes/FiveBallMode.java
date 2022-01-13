package com.team1678.frc2022.auto.modes;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;

public class FiveBallMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // required PathWeaver file paths
    String file_path_a = "paths/FiveBallPaths/5 Ball A.path";
    String file_path_b = "paths/FiveBallPaths/5 Ball B.path";
    String file_path_c = "paths/FiveBallPaths/5 Ball c.path";
    
	// trajectory actions
	SwerveTrajectoryAction driveToIntakeFirstTwoAndShoot;
	SwerveTrajectoryAction driveToIntakeAtTerminal;
	SwerveTrajectoryAction driveToShootFromTerminal;

    public FiveBallMode() {

        // define theta controller for robot heading
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        // read trajectories from PathWeaver and generate trajectory actions
		Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.defaultConfig);
        driveToIntakeFirstTwoAndShoot = new SwerveTrajectoryAction(traj_path_a,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            s_Swerve::setModuleStates);

        Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.defaultConfig);
        driveToIntakeAtTerminal = new SwerveTrajectoryAction(traj_path_b,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            s_Swerve::setModuleStates);

        Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.defaultConfig);
        driveToShootFromTerminal = new SwerveTrajectoryAction(traj_path_c,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            s_Swerve::setModuleStates);
				
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running five ball mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(driveToIntakeFirstTwoAndShoot.getInitialPose())));

        runAction(driveToIntakeFirstTwoAndShoot);
        runAction(driveToIntakeAtTerminal);
        runAction(driveToShootFromTerminal);
        
        System.out.println("Finished auto!");
    }
}
