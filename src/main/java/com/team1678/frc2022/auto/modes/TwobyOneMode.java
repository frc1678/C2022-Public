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

public class TwobyOneMode extends AutoModeBase {

   // Swerve instance 
   private final Swerve mSwerve = Swerve.getInstance();
   private final Superstructure mSuperstructure = Superstructure.getInstance(); 

   // required PathWeaver file paths
   String file_path_a = "paths/TwoBallPaths/2 by 2 A.path";
   String file_path_b = "paths/TwoBallPaths/2 by 2 B.path";
   String file_path_c = "paths/TwoBallPaths/2 by 1 C.path";

   //trajectory actions
   SwerveTrajectoryAction driveToIntakeSecondShotCargo;
   SwerveTrajectoryAction driveToShotPose;
   SwerveTrajectoryAction driveToIntakeLeftEjectCargo;
   
   public TwobyOneMode() {

       SmartDashboard.putBoolean("Auto Finished", false);

       // define theta controller for robot heading
       var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
       thetaController.enableContinuousInput(-Math.PI, Math.PI);
       
       // read trajectories from PathWeaver and generate trajectory actions
       Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.defaultSpeedConfig);
       driveToIntakeSecondShotCargo = new SwerveTrajectoryAction(traj_path_a,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(135.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

        // read trajectories from PathWeaver and generate trajectory actions
       Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.defaultSpeedConfig);
       driveToShotPose = new SwerveTrajectoryAction(traj_path_b,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(135.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

       Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.defaultSpeedConfig);
       driveToIntakeLeftEjectCargo = new SwerveTrajectoryAction(traj_path_c,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(5.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);
   }

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running two by two mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

    // start intaking
    runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

    // wait for modules to align
    runAction(new WaitAction(0.5));

    // drive to intake our second alliance cargo
    runAction(driveToIntakeSecondShotCargo);

    // start spinning up for shot
    runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));
    
    // start vision aiming to align drivetrain to target
    runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

    // drive to closer shot pose
    runAction(driveToShotPose);

    // shoot preloaded and first cargo
    runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
    runAction(new WaitAction(1.0));
    runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

    // start ejecting cargo
    runAction(new LambdaAction(() -> mSuperstructure.setWantEject(false, true)));

    // stop vision aiming to control robot heading
    runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

    // run trajectory to drive to third cargo
    runAction(driveToIntakeLeftEjectCargo);

    // wait to outtake third cargo
    runAction(new WaitAction(2.0));
    
    // stop ejecting cargo
    runAction(new LambdaAction(() -> mSuperstructure.setWantEject(false, false)));

    // ready for teleop
    runAction(new LambdaAction(() -> mSuperstructure.setInitialTeleopStates()));

    System.out.println("Finished auto!");
    SmartDashboard.putBoolean("Auto Finished", true);

    }

    @Override
    public Pose2d getStartingPose() {
        return driveToIntakeSecondShotCargo.getInitialPose();
    }

}
