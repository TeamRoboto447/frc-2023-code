// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {

          if (m_driverController.getRawButton(9)) {
            m_robotDrive.drive(
                2.5,
                0,
                0,
                true);
          } else if (m_driverController.getRawButton(10)) {
            m_robotDrive.drive(
                -2.5,
                0,
                0,
                true);
          } else {
            m_robotDrive.drive(
                deadzoneXY(-m_driverController.getX(), 0.05),
                deadzoneXY(m_driverController.getY(), 0.05),
                deadzoneROT(joystickRotate(), 0.25),
                true);
          }

          SmartDashboard.putNumber("X Coord (meters)", m_robotDrive.getPose().getX());
          SmartDashboard.putNumber("Y Coord (meters)", m_robotDrive.getPose().getY());

        }, m_robotDrive));

    PDP.getModule();
  }

  private double buttonRotate() {
    return m_driverController.getRawButton(5) ? 2 : m_driverController.getRawButton(6) ? -2 : 0;
  }

  private double joystickRotate() {
    return -m_driverController.getZ();
  }

  private double deadzoneXY(double val, double deadzone) {
    if (Math.abs(deadzone) > Math.abs(val))
      return 0;
    else
      return val * 10;
  }

  private double deadzoneROT(double val, double deadzone) {
    if (Math.abs(deadzone) > Math.abs(val))
      return 0;
    else
      return val / 3;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(20, 10).setKinematics(DriveConstants.kDriveKinematics);

    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-2, 0.1)),
        new Pose2d(-4, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-4, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(-4.1, -5)),
        new Pose2d(-4, -10, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        0.75,
        0,
        0,
        new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand movementStep1 = new SwerveControllerCommand(
        trajectory1,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    SwerveControllerCommand movementStep2 = new SwerveControllerCommand(
        trajectory2,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    return new SequentialCommandGroup(
        new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),
        movementStep1,
        movementStep2,
        new InstantCommand(() -> m_robotDrive.stopModules()));
  }
}
