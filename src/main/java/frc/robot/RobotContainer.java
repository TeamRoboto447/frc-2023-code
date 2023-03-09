// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.MoveArmToLimit;
import frc.robot.commands.MoveArmToPosition;
import frc.robot.commands.SetGrabber;
import frc.robot.commands.SetGrabberExtension;
import frc.robot.commands.SetGrabberWithIntake;
import frc.robot.commands.WaitForInput;
import frc.robot.commands.MoveArmToLimit.Limit;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.AutonUtils;
import frc.robot.utils.Toggle;
import frc.robot.utils.AutonUtils.Script;
import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_robotArm = new ArmSubsystem();
  private final PowerDistribution PDP = new PowerDistribution(0, ModuleType.kCTRE);

  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  JoystickControl m_joystickControl = new JoystickControl(m_driverController, 0.25, 0.25, -10, -0.333);

  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  POVToAxis m_operatorPOVAxis = new POVToAxis(m_operatorController, -0.75, 0.75, -0.75, 0.75); // controller, xMin,
                                                                                               // xMax, yMin, yMax

  Toggle leftBumperToggle = new Toggle(false);
  Toggle rightBumperToggle = new Toggle(false);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(() -> {
          m_robotDrive.setBrakeMode(false);

          if (m_driverController.getRawButton(5)) {
            m_robotDrive.drive(
                -2.5,
                0,
                0,
                true);

          } else if (m_driverController.getRawButton(6)) {
            m_robotDrive.drive(
                2.5,
                0,
                0,
                true);
          } else if (m_driverController.getRawButton(3)) {
            m_joystickControl.setEnableXYControl(enableXY());

            m_robotDrive.drive(
                m_joystickControl.getX(),
                m_joystickControl.getY(),
                0.1,
                true);
          } else if (m_driverController.getRawButton(4)) {
            m_joystickControl.setEnableXYControl(enableXY());

            m_robotDrive.drive(
                m_joystickControl.getX(),
                m_joystickControl.getY(),
                -0.1,
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

          updateSmartdashboard();

        }, m_robotDrive));

    m_robotArm.setDefaultCommand(
        new RunCommand(() -> {

          // m_robotArm.rawMoveHorizontal(deadzone(-m_operatorController.getRightX() / 1,
          // 0.25));
          // m_robotArm.teleopMoveVertical(deadzone(-m_operatorController.getLeftY() / 1,
          // 0.25));

          m_robotArm.rawMoveHorizontal(m_operatorPOVAxis.getY());
          m_robotArm.teleopMoveVertical(m_operatorPOVAxis.getX());

          if (m_operatorController.getRightTriggerAxis() > 0.25)
            m_robotArm.rawIntakeGrabber(m_operatorController.getRightTriggerAxis());
          else if (m_operatorController.getLeftTriggerAxis() > 0.25)
            m_robotArm.rawIntakeGrabber(-m_operatorController.getLeftTriggerAxis());
          else
            m_robotArm.rawIntakeGrabber(0);

          leftBumperToggle.runToggle(m_operatorController.getLeftBumper());
          if (leftBumperToggle.getState())
            m_robotArm.extend();
          else
            m_robotArm.retract();

          rightBumperToggle.runToggle(m_operatorController.getRightBumper());
          if (rightBumperToggle.getState())
            m_robotArm.open();
          else
            m_robotArm.close();

        }, m_robotArm));

    PDP.getModule();
  }

  // private boolean axisAsButton(double val) {
  // return Math.abs(val) > 0.5;
  // }

  private boolean enableROT() {
    return m_driverController.getRawButton(1);
  }

  private boolean enableXY() {
    return m_driverController.getRawButton(2);
  }

  private void updateSmartdashboard() {
    SmartDashboard.putNumber("Processing X Coord", Units.metersToFeet(m_robotDrive.getPose().getX()));
    SmartDashboard.putNumber("Processing Y Coord", Units.metersToFeet(m_robotDrive.getPose().getY()));

    SmartDashboard.putNumber("Gyro Angle", m_robotDrive.getHeading());

    SmartDashboard.putNumber("Vertical Arm Encoder", m_robotArm.getVertEncoder());
    SmartDashboard.putNumber("Horizontal Arm Encoder", m_robotArm.getHorizontalEncoder());
  }

  private boolean shouldAbortCommand() {
    return m_operatorController.getBackButton();
  }

  private void configureBindings() {
    Trigger aButton = new Trigger(() -> m_operatorController.getAButton());
    Trigger bButton = new Trigger(() -> m_operatorController.getBButton());
    Trigger xButton = new Trigger(() -> m_operatorController.getXButton());
    Trigger yButton = new Trigger(() -> m_operatorController.getYButton());

    Trigger freezeRobot = new Trigger(() -> m_driverController.getRawButton(8));

    freezeRobot.onTrue(
        new FunctionalCommand(
            () -> {
            }, // Init, don't do anything
            () -> { // Execute
              m_robotArm.stop();
              m_robotDrive.drive(0, 0, 0, false);
              m_robotDrive.hold();
            },
            inturrupted -> { // On End
              m_robotDrive.allowMovement();
            },
            () -> m_driverController.getRawButton(7), // Whether to end or not
            m_robotDrive,
            m_robotArm));

    aButton.onTrue(
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new SetGrabberWithIntake(m_robotArm, false, -1),
                //new MoveArmToPosition(m_robotArm, Double.NaN, Double.NaN, -0.2),
                //new MoveArmToPosition(m_robotArm, 111, Double.NaN, -1),
                new MoveArmToLimit(m_robotArm, Limit.TOP_VERTICAL, Limit.NO_CHANGE, -1),
                new MoveArmToLimit(m_robotArm, Limit.NO_CHANGE, Limit.CLOSE_HORIZONTAL, 0),
                new MoveArmToLimit(m_robotArm, Limit.NO_CHANGE, Limit.NO_CHANGE, 0)),
                //new MoveArmToPosition(m_robotArm, Double.NaN, 0, 0)),
            new WaitForInput(this::shouldAbortCommand)));

    bButton.onTrue(
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new SetGrabberExtension(m_robotArm, false),
                new MoveArmToPosition(m_robotArm, Double.NaN, 116, 0),
                new SetGrabber(m_robotArm, true),
                new MoveArmToLimit(m_robotArm, Limit.BOTTOM_VERTICAL, Limit.NO_CHANGE, -1),
                new SetGrabber(m_robotArm, false),
                new MoveArmToLimit(m_robotArm, Limit.NO_CHANGE, Limit.NO_CHANGE, -1),
                new MoveArmToLimit(m_robotArm, Limit.NO_CHANGE, Limit.NO_CHANGE, 0)),
            new WaitForInput(this::shouldAbortCommand)));

   // xButton.onTrue(
   //     new ParallelRaceGroup(
   //         new SequentialCommandGroup(
   //             new SetGrabberExtension(m_robotArm, true),
   //             new MoveArmToLimit(m_robotArm, Limit.TOP_VERTICAL, Limit.NO_CHANGE, 0),
   //             new WaitForInput(() -> m_operatorController.getRightBumper()),
   //             new MoveArmToPosition(m_robotArm, 0, 0, 0)),
   //         new WaitForInput(this::shouldAbortCommand)));

   // yButton.onTrue(
   //     new ParallelRaceGroup(new SequentialCommandGroup(
   //         new SetGrabberExtension(m_robotArm, true),
   //         new WaitForInput(() -> m_operatorController.getRightBumper()),
   //         new MoveArmToPosition(m_robotArm, 0, 0, 0)),
   //         new WaitForInput(this::shouldAbortCommand)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @param script
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Script script) {
    m_robotDrive.updateEstimationFromVision();
    return new ParallelRaceGroup( // Parallel Race Group runs commands in parallel, when one ends, they all end.
        new RunCommand(() -> updateSmartdashboard()), // This just runs a command that never ends and just updates the
                                                      // dashboard during auto
        AutonUtils.getCommandScript(this, script)); // This gets the autonomous script
                                                                               // (usually a sequential command group)
  }

  protected double deadzone(double val, double deadzone) {
    if (Math.abs(deadzone) > Math.abs(val))
      return 0;
    else
      return val;
  }

  private class POVToAxis {
    private final XboxController joystick;
    private final double xMin;
    private final double xMax;
    private final double yMin;
    private final double yMax;

    public POVToAxis(XboxController joystick, double xMin, double xMax, double yMin, double yMax) {
      this.joystick = joystick;
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
    }

    public double getX() {
      double pov = this.joystick.getPOV();
      if (pov == 315 || pov == 0 || pov == 45)
        return this.yMax;
      if (pov == 135 || pov == 180 || pov == 225)
        return this.yMin;
      return 0;
    }

    public double getY() {
      double pov = this.joystick.getPOV();
      if (pov == 45 || pov == 90 || pov == 135)
        return this.xMin;
      if (pov == 225 || pov == 270 || pov == 315)
        return this.xMax;
      return 0;
    }
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
