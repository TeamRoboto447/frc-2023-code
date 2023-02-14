// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SetGrabber extends InstantCommand {
  private final ArmSubsystem armSubsystem;
  private final boolean open;
  /** Creates a new SetGrabber. */
  public SetGrabber(ArmSubsystem armSubsystem, boolean open) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.open = open;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.open) this.armSubsystem.open();
    else this.armSubsystem.close();
  }
}
