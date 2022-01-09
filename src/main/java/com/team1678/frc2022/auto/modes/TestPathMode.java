package com.team1678.frc2022.auto.modes;

import java.io.IOException;
import java.nio.file.Path;

import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.WaypointReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TestPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // required PathWeaver trajectory paths
    String path = "paths/test.path";
    
	// trajectories
	Trajectory testTrajectory;
	SwerveTrajectoryAction testTrajectoryAction;

    public TestPathMode() {
    
        try {

          Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(path);
          TrajectoryGenerator.ControlVectorList cv_one = WaypointReader.getControlVectors(traj_path);
		  testTrajectory = TrajectoryGenerator.generateTrajectory(cv_one, Constants.AutoConstants.defaultConfig);
		  
        } catch (IOException ex) {
		  DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
		}
		
        var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        testTrajectoryAction = new SwerveTrajectoryAction(testTrajectory,
                s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
                s_Swerve::setModuleStates);
				
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running test mode auto!");

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> s_Swerve.resetOdometry(testTrajectoryAction.getInitialPose())));

        runAction(testTrajectoryAction);
        
        System.out.println("Finished auto!");
    }
}
