// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmToPosition extends CommandBase {
  private final ArmSubsystem armSubsystem;
  private final double targetHeight;
  private final double targetDist;
  private final double targetRot;
  private boolean verticalDone = false;
  private boolean horizontalDone = false;
  private boolean rotationDone = false;
  /** Creates a new ExtendAndLift. */
  public MoveArmToPosition(ArmSubsystem aSubsystem, double tHeight, double tDist, double tRot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = aSubsystem;
    this.targetHeight = tHeight;
    this.targetDist = tDist;
    this.targetRot = tRot;
    addRequirements(aSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.armSubsystem.setMaxArmSpeeds(0.75);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.armSubsystem.goToVertical(targetHeight);
    this.armSubsystem.goToHorizontal(targetDist);
    this.armSubsystem.goToRotation(targetRot);

    this.verticalDone = Double.isNaN(targetHeight) ? true : Math.abs(this.armSubsystem.getVertEncoder() - this.targetHeight) < ArmConstants.verticalPIDTolerance;
    this.horizontalDone = Double.isNaN(targetDist) ? true : Math.abs(this.armSubsystem.getHorizontalEncoder() - this.targetDist) < ArmConstants.horizontalPIDTolerance;
    this.rotationDone = Double.isNaN(targetRot) ? true : Math.abs(this.armSubsystem.getRotationalEncoder() - this.targetRot) < ArmConstants.rotationalPIDTolerance;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.setMaxArmSpeeds(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.verticalDone && this.horizontalDone && this.rotationDone;
  }
}
