// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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
  private final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

  private final DigitalInput autonomousSelector1 = new DigitalInput(0);

  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  JoystickControl m_joystickControl = new JoystickControl(m_driverController, 0.1, 0.25, 10, 0.333);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {

          if (m_driverController.getRawButton(5)) {
            m_robotDrive.drive(
                2.5,
                0,
                0,
                true);

          } else if (m_driverController.getRawButton(6)) {
            m_robotDrive.drive(
                -2.5,
                0,
                0,
                true);
          } else {

            m_joystickControl.setEnableXYControl(enableXY());
            m_joystickControl.setEnableROTControl(enableROT());

            m_robotDrive.drive(
                m_joystickControl.getX(),
                m_joystickControl.getY(),
                m_joystickControl.getROT(),
                true);
          }

          SmartDashboard.putNumber("X Coord (meters)", m_robotDrive.getPose().getX());
          SmartDashboard.putNumber("Y Coord (meters)", m_robotDrive.getPose().getY());

        }, m_robotDrive));

    m_robotArm.setDefaultCommand(
        new RunCommand(() -> {

          m_robotArm.moveHorizontal(deadzone(m_operatorController.getLeftX(), 0.25));
          m_robotArm.moveVertical(deadzone(m_operatorController.getLeftY(), 0.25));

          m_robotArm.moveRotation(deadzone(m_operatorController.getRightX(), 0.25));

          if(m_operatorController.getLeftBumper())
            m_robotArm.extend();
          else
            m_robotArm.retract();

          if(m_operatorController.getRightBumper())
            m_robotArm.open();
          else
            m_robotArm.close();

        }, m_robotArm));

    PDP.getModule();
  }

  private boolean enableROT() {
    return m_driverController.getRawButton(1);
  }

  private boolean enableXY() {
    return m_driverController.getRawButton(2);
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    if (autonomousSelector1.get()) {

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

    } else {

      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(20, 10).setKinematics(DriveConstants.kDriveKinematics);

      Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
              new Translation2d(-2, 0.1)),
          new Pose2d(-4, 0, Rotation2d.fromDegrees(0)),
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

      return new SequentialCommandGroup(
          new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())),
          movementStep1,
          new InstantCommand(() -> m_robotDrive.stopModules()));
    }
  }

  protected double deadzone(double val, double deadzone) {
    if (Math.abs(deadzone) > Math.abs(val))
      return 0;
    else
      return val;
  }

  private class JoystickControl {
    private final Joystick joystick;
    private final double deadzoneXY;
    private final double deadzoneROT;
    private final double multiplierXY;
    private final double multiplierROT;
    private boolean enableXY;
    private boolean enableROT;

    public JoystickControl(Joystick joystick, double deadzoneXY, double deadzoneROT, double multiplierXY,
        double multiplierROT) {
      this.joystick = joystick;
      this.deadzoneXY = deadzoneXY;
      this.deadzoneROT = deadzoneROT;
      this.multiplierXY = multiplierXY;
      this.multiplierROT = multiplierROT;
    }

    public double getX() {
      return this.enableXY ? (deadzone(-this.joystick.getX(), this.deadzoneXY) * this.multiplierXY) : 0;
    }

    public double getY() {
      return this.enableXY ? (deadzone(this.joystick.getY(), this.deadzoneXY) * this.multiplierXY) : 0;
    }

    public double getROT() {
      return this.enableROT ? (deadzone(this.joystick.getZ(), this.deadzoneROT) * this.multiplierROT) : 0;
    }

    public void setEnableXYControl(boolean enabled) {
      this.enableXY = enabled;
    }

    public void setEnableROTControl(boolean enabled) {
      this.enableROT = enabled;
    }
  }
}
