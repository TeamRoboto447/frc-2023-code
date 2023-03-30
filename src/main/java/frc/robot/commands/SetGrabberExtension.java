// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class SetGrabberExtension extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final boolean extend;
  private final Timer delay = new Timer();
  private final double waitTime;
  /** Creates a new SetGrabber. */
  public SetGrabberExtension(ArmSubsystem armSubsystem, boolean extend, double delayBeforeMovingOn) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.extend = extend;
    this.waitTime = delayBeforeMovingOn;
    addRequirements(armSubsystem);
  }

  public SetGrabberExtension(ArmSubsystem armSubsystem, boolean extend) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.extend = extend;
    this.waitTime = 1;
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
    this.armSubsystem.holdAll();
    System.out.println("Set Extension: " + this.extend);
  }

  @Override
  public boolean isFinished() {
    return this.delay.get() > this.waitTime;
  }
}
