// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.SetGrabber;
import frc.robot.commands.SetGrabberExtension;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutonUtils;
import frc.robot.subsystems.ArmSubsystem;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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

  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  JoystickControl m_joystickControl = new JoystickControl(m_driverController, 0.25, 0.25, -10, -0.333);
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


          if(m_operatorController.getXButton()) {
            m_robotArm.rawMoveHorizontal(deadzone(m_operatorController.getRightY()/4, 0.25));
            m_robotArm.goToVertical(ArmConstants.verticalRange);
            m_robotArm.rawRotateGrabber(deadzone(m_operatorController.getRightX()/4, 0.25));
          }
          else if(m_operatorController.getYButton()) {
            m_robotArm.rawMoveHorizontal(deadzone(m_operatorController.getRightY()/4, 0.25));
            m_robotArm.goToVertical(0);
            m_robotArm.rawRotateGrabber(deadzone(m_operatorController.getRightX()/4, 0.25));
            m_robotArm.open();
          }
          else if(m_operatorController.getBButton()) {
            m_robotArm.rawMoveHorizontal(deadzone(m_operatorController.getRightY()/4, 0.25));
            m_robotArm.goToVertical(ArmConstants.verticalRange/4);
            m_robotArm.rawRotateGrabber(deadzone(m_operatorController.getRightX()/4, 0.25));
          }
          else if(m_operatorController.getAButton()) {
            m_robotArm.rawMoveHorizontal(deadzone(m_operatorController.getRightY()/4, 0.25));
            m_robotArm.goToVertical(ArmConstants.verticalRange/2);
            m_robotArm.rawRotateGrabber(deadzone(m_operatorController.getRightX()/4, 0.25));
          }
          else {
            m_robotArm.rawMoveHorizontal(deadzone(-m_operatorController.getRightY()/4, 0.25));
            m_robotArm.rawMoveVertical(deadzone(-m_operatorController.getLeftY()/4, 0.25));
            m_robotArm.rawRotateGrabber(deadzone(m_operatorController.getRightX()/4, 0.25));
          }

          if(m_operatorController.getLeftBumper())
            m_robotArm.extend();
          else if (m_operatorController.getRightBumper())
            m_robotArm.retract();

          if(m_operatorController.getBackButton())
            m_robotArm.open();
          else if(m_operatorController.getStartButton())
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
 

  // boolean demoAuto = true;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    Trajectory traj1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(AutonUtils.rotationOffsetCorrection(0))),
      List.of(), 
      new Pose2d(0, AutoConstants.unitsPerMeter/2, Rotation2d.fromDegrees(AutonUtils.rotationOffsetCorrection(90))), // Field orientation, drove left
      AutoConstants.trajectoryConfig);

    FollowTrajectory movement1 = new FollowTrajectory(m_robotDrive, traj1, false);

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(AutonUtils.getStartingPose(traj1, m_robotDrive))),
      movement1,
      new InstantCommand(() -> m_robotDrive.stopModules())
    );
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
