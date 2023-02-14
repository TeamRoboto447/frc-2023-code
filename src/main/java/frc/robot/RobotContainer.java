// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.GetAutonomousScript;
import frc.robot.subsystems.ArmSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.AutoConstants.trajectoryConfig;
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

          m_robotArm.rotateGrabber(deadzone(m_operatorController.getRightX(), 0.25));

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
      Trajectory trajectory1 = GetAutonomousScript.getTrajectory(GetAutonomousScript.Script.DEMO_SCRIPT_1, 1); // Get first movement trajectory based on what script and what step
      FollowTrajectory movementStep1 = new FollowTrajectory(m_robotDrive, trajectory1, false); // Use first movement trajectory to make a path following command
      Trajectory trajectory2 = GetAutonomousScript.getTrajectory(GetAutonomousScript.Script.DEMO_SCRIPT_1, 2); // Get second movement trajectory based on what script and what step
      FollowTrajectory movementStep2 = new FollowTrajectory(m_robotDrive, trajectory2, true); // Use second movement trajectory to make a path following command
      
      return new SequentialCommandGroup( // Return a sequential command group (Does the listed commands in order)
          new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())), //reset position to match expected starting position
          movementStep1, // do first movement
          movementStep2, // do second movement
          new InstantCommand(() -> m_robotDrive.stopModules())); // stop the robot

    } else {

      Trajectory trajectory1 = GetAutonomousScript.getTrajectory(GetAutonomousScript.Script.DEMO_SCRIPT_2, 1); // Get movement trajectory based on what script and what step
      SwerveControllerCommand movementStep1 = new FollowTrajectory(m_robotDrive, trajectory1, true); // Use movement trajectory to make a path following command
      
      return new SequentialCommandGroup( // Return a sequential command group (Does the listed commands in order)
          new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory1.getInitialPose())), //reset position to match expected starting position
          movementStep1, // do movement
          new InstantCommand(() -> m_robotDrive.stopModules())); // stop the robot
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
      return this.enableROT ? -(deadzone(this.joystick.getZ(), this.deadzoneROT) * this.multiplierROT) : 0;
    }

    public void setEnableXYControl(boolean enabled) {
      this.enableXY = enabled;
    }

    public void setEnableROTControl(boolean enabled) {
      this.enableROT = enabled;
    }
  }
}
