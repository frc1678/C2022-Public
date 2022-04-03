package com.team1678.frc2022.auto.actions;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;


import java.util.function.Consumer;
import java.util.function.Supplier;

import com.team1678.frc2022.subsystems.Limelight;
import com.team1678.frc2022.subsystems.Superstructure;

/**
 * An action that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This action outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 */
@SuppressWarnings("MemberName")
public class SwerveTrajectoryAction implements Action {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;
  private final Supplier<Boolean> m_wantsVisionAlign;

  // required subsystem instances
  Limelight mLimelight = Limelight.getInstance();

  /**
   * Constructs a new SwerveTrajectoryAction that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputModuleStates The raw output module states from the position controllers.
   */
  @SuppressWarnings("ParameterName")
  public SwerveTrajectoryAction(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      Supplier<Boolean> wantsVisionAlign,
      Consumer<SwerveModuleState[]> outputModuleStates) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "SwerveTrajectoryAction");
    m_pose = requireNonNullParam(pose, "pose", "SwerveTrajectoryAction");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "SwerveTrajectoryAction");

    m_controller =
        new HolonomicDriveController(
            requireNonNullParam(xController, "xController", "SwerveTrajectoryAction"),
            requireNonNullParam(yController, "xController", "SwerveTrajectoryAction"),
            requireNonNullParam(thetaController, "thetaController", "SwerveTrajectoryAction"));

    m_outputModuleStates =
        requireNonNullParam(outputModuleStates, "frontLeftOutput", "SwerveTrajectoryAction");

    m_desiredRotation =
        requireNonNullParam(desiredRotation, "desiredRotation", "SwerveTrajectoryAction");

    m_wantsVisionAlign = 
        requireNonNullParam(wantsVisionAlign, "wantsVisionAlign", "SwerveTrajectoryAction");
  }

  /**
   * Constructs a new SwerveControllerAction that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   */
  @SuppressWarnings("ParameterName")
  public SwerveTrajectoryAction(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        () -> false, // no vision align
        outputModuleStates);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }

  @Override
  public void start() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void update() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);
    Rotation2d desiredRotation = new Rotation2d();

    if (m_wantsVisionAlign.get()) {
      if (mLimelight.hasTarget()) {
        desiredRotation = Rotation2d.fromDegrees(m_pose.get().getRotation().getDegrees() - mLimelight.getOffset()[0]);
      } else {
        desiredRotation = Rotation2d.fromDegrees(Superstructure.getInstance().getRealAimingParameters().get().getVehicleToGoalRotation().getWPIRotation2d().getDegrees() + 180.0);
        System.out.println(Superstructure.getInstance().getRealAimingParameters().get().getVehicleToGoalRotation().getWPIRotation2d().getDegrees() + 180.0);
      }
    } else {
      desiredRotation = m_desiredRotation.get();
    }

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, desiredRotation);
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void done() {
    m_timer.stop();
    m_outputModuleStates.accept(m_kinematics.toSwerveModuleStates((ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0)))));
  }

  // get initial pose
  public Pose2d getInitialPose() {
    return m_trajectory.getInitialPose();
  }

}
