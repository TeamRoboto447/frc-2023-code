// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class SetGrabberExtensionWithIntake extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final boolean extend;
  private final double intakeSpeed;
  private final Timer delay = new Timer();

  public SetGrabberExtensionWithIntake(ArmSubsystem armSubsystem, boolean extend, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.extend = extend;
    this.intakeSpeed = intakeSpeed;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.delay.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.extend) this.armSubsystem.extend();
    else this.armSubsystem.retract();
    this.armSubsystem.holdHorizontal();
    this.armSubsystem.holdVertical();
    this.armSubsystem.rawIntakeGrabber(this.intakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return this.delay.get() > 1;
  }
}
