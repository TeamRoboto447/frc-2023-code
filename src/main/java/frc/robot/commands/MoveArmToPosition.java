// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double targetHeight;
  private final double targetDist;
  private boolean verticalDone = false;
  private boolean horizontalDone = false;
  /** Creates a new ExtendAndLift. */
  public MoveArmToPosition(ArmSubsystem aSubsystem, double tHeight, double tDist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = aSubsystem;
    this.targetHeight = tHeight;
    this.targetDist = tDist;
    addRequirements(aSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.verticalDone = this.armSubsystem.goToVertical(targetHeight);
    this.horizontalDone = this.armSubsystem.goToHorizontal(targetDist);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return verticalDone && horizontalDone;
  }
}
