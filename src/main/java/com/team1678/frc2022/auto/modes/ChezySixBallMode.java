package com.team1678.frc2022.auto.modes;

import java.util.List;

import com.team1678.frc2022.Constants;
import com.team1678.frc2022.auto.AutoModeEndedException;
import com.team1678.frc2022.auto.AutoTrajectoryReader;
import com.team1678.frc2022.auto.actions.LambdaAction;
import com.team1678.frc2022.auto.actions.RaceAction;
import com.team1678.frc2022.auto.actions.SeriesAction;
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

public class ChezySixBallMode extends AutoModeBase {

        // Swerve instance
        private final Swerve mSwerve = Swerve.getInstance();
        private final Superstructure mSuperstructure = Superstructure.getInstance();

        // required PathWeaver file paths
        String file_path_a = "paths/ChezySixBallPaths/6 Chezy A.path";
        String file_path_b = "paths/ChezySixBallPaths/6 Chezy B.path";
        String file_path_c = "paths/ChezySixBallPaths/6 Chezy C.path";
        String file_path_d = "paths/ChezySixBallPaths/6 Chezy D.path";
        String file_path_e = "paths/ChezySixBallPaths/6 Chezy E.path";
        String file_path_f = "paths/ChezySixBallPaths/6 Chezy F.path";

        // trajectories
        private Trajectory traj_path_a;
        private Trajectory traj_path_b;
        private Trajectory traj_path_c;
        private Trajectory traj_path_d;
        private Trajectory traj_path_e;
        private Trajectory traj_path_f;

        // trajectory actions
        SwerveTrajectoryAction driveToFirstShot;
        SwerveTrajectoryAction driveToIntakeThirdCargo;
        SwerveTrajectoryAction driveToIntakeFourthCargo;
        SwerveTrajectoryAction driveToIntakeAtTerminal;
        SwerveTrajectoryAction driveToHumanPlayerWait;
        SwerveTrajectoryAction driveToSecondShot;

        public ChezySixBallMode() {

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

                driveToFirstShot = new SwerveTrajectoryAction(traj_path_a,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(270.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Drive to lineup to third cargo
                traj_path_b = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_b,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                Constants.AutoConstants.kSlowAccelerationMetersPerSecondSquared,
                                                0.0,
                                                0.0));

                driveToIntakeThirdCargo = new SwerveTrajectoryAction(traj_path_b,
                                mSwerve::getPose, Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                () -> Rotation2d.fromDegrees(270.0),
                                mSwerve::getWantAutoVisionAim,
                                mSwerve::setModuleStates);

                // Intake third cargo
                traj_path_c = AutoTrajectoryReader.generateTrajectoryFromFile(file_path_c,
                                Constants.AutoConstants.createConfig(
                                                Constants.AutoConstants.kSlowSpeedMetersPerSecond,
                                                1.7,
                                                0.0,
                                                0.0));

                driveToIntakeFourthCargo = new SwerveTrajectoryAction(traj_path_c,
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
                                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
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
                                                5.0,
                                                4.0,
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
        }

        @Override
        protected void routine() throws AutoModeEndedException {
                System.out.println("Running five ball mode auto!");
                SmartDashboard.putBoolean("Auto Finished", false);

                // disable auto ejecting
                runAction(new LambdaAction(() -> mSuperstructure.setEjectDisable(false)));

                runAction(new RaceAction(
                                new SeriesAction(List.of(
                                        driveToFirstShot
                                )),

                                new SeriesAction(List.of(
                                        new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)),
                                        new LambdaAction(() -> mSuperstructure.setWantPrep(true))
                                ))));

                                
                // shoot both preloads
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(true)));
                runAction(new WaitAction(1.0));
                runAction(new LambdaAction(() -> mSuperstructure.setWantShoot(false)));

                // start run intake
                runAction(new LambdaAction(() -> mSuperstructure.setWantIntake(true)));

                runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(false)));

                // run trajectory to intake third cargo
                runAction(driveToIntakeThirdCargo);

                runAction(new LambdaAction(() -> mSwerve.setWantAutoVisionAim(true)));

                // run trajectory to intake fourth cargo
                runAction(driveToIntakeFourthCargo);

                // shoot third and fourth cargo
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

                System.out.println("Finished auto!");
                SmartDashboard.putBoolean("Auto Finished", true);
        }

        @Override
        public Pose2d getStartingPose() {
                return driveToFirstShot.getInitialPose();
        }
}
