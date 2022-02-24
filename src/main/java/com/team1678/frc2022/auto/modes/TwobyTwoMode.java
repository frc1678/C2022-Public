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

public class TwoByTwoMode extends AutoModeBase {

   // Swerve instance 
   private final Swerve mSwerve = Swerve.getInstance();
   private final Superstructure mSuperstructure = Superstructure.getInstance(); 

   // required PathWeaver file paths
   String file_path_a = "paths/TwoBallPaths/2 by 2 A.path";
   String file_path_b = "paths/TwoBallPaths/2 by 2 B.path";
   String file_path_c = "paths/TwoBallPaths/2 by 2 C.path";

   //trajectory actions
   SwerveTrajectoryAction driveToIntakeFirstShootCargo;
   SwerveTrajectoryAction driveToIntakeSecondEjectCargo;
   SwerveTrajectoryAction driveToIntakeThirdEjectCargo;
   
   public TwoByTwoMode() {

       SmartDashboard.putBoolean("Auto Finished", false);

       // define theta controller for robot heading
       var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                                        Constants.AutoConstants.kThetaControllerConstraints);
       thetaController.enableContinuousInput(-Math.PI, Math.PI);
       
       // read trajectories from PathWeaver and generate trajectory actions
       Trajectory traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a, Constants.AutoConstants.defaultSpeedConfig);
       driveToIntakeFirstShootCargo = new SwerveTrajectoryAction(traj_path_a,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(235.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

       Trajectory traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b, Constants.AutoConstants.defaultSpeedConfig);
       driveToIntakeSecondEjectCargo = new SwerveTrajectoryAction(traj_path_b,
                                                           mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                                           new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                                           new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                                           thetaController,
                                                           () -> Rotation2d.fromDegrees(320.0),
                                                           mSwerve::getWantAutoVisionAim,
                                                           mSwerve::setModuleStates);

       Trajectory traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c, Constants.AutoConstants.defaultSpeedConfig);
       driveToIntakeThirdEjectCargo = new SwerveTrajectoryAction(traj_path_c,
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
        System.out.println("Running two by two left mode auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> mSwerve.resetOdometry(new Pose2d(
            driveToIntakeFirstShootCargo.getInitialPose().getX(),
            driveToIntakeFirstShootCargo.getInitialPose().getY(),
            Rotation2d.fromDegrees(235)))));

        // start spinning up for shot
        runAction(new LambdaAction(() -> mSuperstructure.setWantPrep(true)));
   
        // start intaking
        runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

        // start vision aiming to align drivetrain to target
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

        // drive to intake our alliance cargo for shot
        runAction(driveToIntakeFirstShootCargo);

        // wait to settle
        runAction(new WaitAction(1.0));

        // shoot preloaded and first cargo
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
        runAction(new WaitAction(1.0));
        runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

        // stop vision aiming to control robot heading
        runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

        // run trajectory to drive to second cargo
        runAction(driveToIntakeSecondEjectCargo);
    
        // wait to outtake second cargo
        //runAction(new LambdaAction(() -> mSuperstructure.setWantEject(true)));
        runAction(new WaitAction(2.0));
        // runAction(new LambdaAction(() -> mSuperstructure.setWantEject(false)));

        // run trajectory to drive to third cargo
        runAction(driveToIntakeThirdEjectCargo);

        // wait to outtake third cargo
        // runAction(new LambdaAction(() -> mSuperstructure.setWantEject(true)));
        runAction(new WaitAction(2.0));
        // runAction(new LambdaAction(() -> mSuperstructure.setWantEject(false)));

        System.out.println("Finished auto!");
            SmartDashboard.putBoolean("Auto Finished", true);

    }

}