package com.team1678.frc2022.auto.modes;

import java.util.List;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.ShuffleBoardInteractions;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.ParallelAction;
import com.team1678.frc2022.auto.actions.RaceAction;
import com.team1678.frc2022.auto.actions.SeriesAction;
import com.team1678.frc2022.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2022.auto.actions.WaitAction;
import com.team1678.frc2022.subsystems.Superstructure;
import com.team1678.frc2022.subsystems.Swerve;
import com.team254.lib.geometry.Pose2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
        String file_path_f = "paths/FiveBallPaths/5 Ball F.path";

        // trajectories
        private Trajectory traj_path_a;
        private Trajectory traj_path_b;
        private Trajectory traj_path_c;
        private Trajectory traj_path_d;
        private Trajectory traj_path_e;
        private Trajectory traj_path_f;

        // trajectory actions
        SwerveTrajectoryAction driveToIntakeSecondCargo;
        SwerveTrajectoryAction driveToFirstShot;
        SwerveTrajectoryAction driveToIntakeThirdCargo;
        SwerveTrajectoryAction driveToIntakeAtTerminal;
        SwerveTrajectoryAction driveToHumanPlayerWait;
        SwerveTrajectoryAction driveToSecondShot;

        public FiveBallMode() {

                SmartDashboard.putBoolean("Auto Finished", false);

                // define theta controller for robot heading
                var thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                /* CREATE TRAJECTORIES FROM FILES */

                // Intake second cargo
                traj_path_a = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_a,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeSecondCargo = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(265.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to lineup to third cargo
                traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToFirstShot = new SwerveTrajectoryAction(traj_path_b,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(200.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Intake third cargo
                traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeThirdCargo = new SwerveTrajectoryAction(traj_path_c,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(200.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to intake 4th cargo at terminal
                traj_path_d = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_d,
                                Constants.AutoConstants.createConfig(
                                                3.0,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeAtTerminal = new SwerveTrajectoryAction(traj_path_d,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(225.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to human player wait pose
                traj_path_e = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_e,
                                Constants.AutoConstants.createConfig(
                                                3.0,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToHumanPlayerWait = new SwerveTrajectoryAction(traj_path_e,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(225.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to second shot
                traj_path_f = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_f,
                                Constants.AutoConstants.createConfig(
                                                3.0,
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToSecondShot = new SwerveTrajectoryAction(traj_path_f,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(210.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                plotTrajectories();


        }

        @Override
        protected void routine() throws AutoModeEndedException {
                System.out.println("Running five ball mode a auto!");
                SmartDashboard.putBoolean("Auto Finished", false);

                // reset odometry at the start of the trajectory
                runAction(new LambdaAction(() -> mSwerve.resetOdometry(driveToIntakeSecondCargo.getInitialPose())));

                // runAction(new LambdaAction(() -> mSwerve.setModuleStates(
			// Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates((
			// ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))))));

                runAction(new RaceAction(
                                new SeriesAction(List.of(
                                                // new WaitAction(0.2),
                                                driveToIntakeSecondCargo)),

                                new SeriesAction(List.of(
                                                new WaitAction(0.1),
                                                new LambdaAction(() -> mSuperstructure.setWantPrep(true)),
                                                new LambdaAction(() -> mSuperstructure.setWantIntake(true))))));


                // start vision aiming
                runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                // run trajectory for third cargo
                runAction(driveToFirstShot);

                // shoot first & second cargo
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                runAction(new WaitAction(1.0));
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // run trajectory to intake third shot cargo
                runAction(driveToIntakeThirdCargo);

                // shoot third cargo
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                runAction(new WaitAction(1.0));
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // stop vision aiming to control robot heading
                runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

                // run trajectory for terminal
                runAction(driveToIntakeAtTerminal);

                runAction(driveToHumanPlayerWait);

                runAction(new WaitAction(0.25));

                // start vision aiming when driving to shot pose
                runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                // run trajectory to drive to second shot pose
                runAction(driveToSecondShot);

                // shoot cargo
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                runAction(new WaitAction(1.0));
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                System.out.println("Finished auto!");
                SmartDashboard.putBoolean("Auto Finished", true);
        }

        public void plotTrajectories() {
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_a, "Traj A");
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_b, "Traj B");
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_c, "Traj C");
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_d, "Traj D");
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_e, "Traj E");
                ShuffleBoardInteractions.getInstance().addTrajectory(traj_path_f, "Traj F");
        }
}
