package com.team1678.frc2022.auto.modes;

import java.util.List;

import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.geometry.Translation2d;

public class TestPathMode extends AutoModeBase {
    
    // Swerve instance 
    private final Swerve s_Swerve = Swerve.getInstance();

    // required PathWeaver trajectory paths
    String path = "paths/test.path";
    
	// trajectories
	SwerveTrajectoryAction testTrajectoryAction;

    public TestPathMode() {

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    
        // read trajectories from PathWeaver and generate trajectory actions
        Trajectory traj_path = AutoTrajectoryReader.generateTrajectoryFromFile(path, Constants.AutoConstants.defaultSpeedConfig);
        testTrajectoryAction = new SwerveTrajectoryAction(traj_path,
                                                            s_Swerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                            thetaController,
                                                            () -> Rotation2d.fromDegrees(0.0),
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
