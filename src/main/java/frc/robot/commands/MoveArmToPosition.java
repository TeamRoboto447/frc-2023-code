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
    this.verticalDone = Double.isNaN(targetHeight) ? true : this.runVert(targetHeight);
    this.horizontalDone = Double.isNaN(targetDist) ? true : this.runHoriz(targetDist);
    this.rotationDone = Double.isNaN(targetRot) ? true : this.runRot(targetRot);
  }

  private boolean runVert(double targetHeight) {
    if(this.armSubsystem.getVertEncoder() < targetHeight) this.armSubsystem.teleopMoveVertical(ArmConstants.vertBangBangSpeed);
    if(this.armSubsystem.getVertEncoder() > targetHeight) this.armSubsystem.teleopMoveVertical(-ArmConstants.vertBangBangSpeed);
    return withinMargin(this.armSubsystem.getVertEncoder(), targetHeight, ArmConstants.verticalPIDTolerance);
  }

  private boolean runHoriz(double targetDist) {
    if(this.armSubsystem.getHorizontalEncoder() < targetDist) this.armSubsystem.rawMoveHorizontal(ArmConstants.horizBangBangSpeed);
    if(this.armSubsystem.getHorizontalEncoder() > targetDist) this.armSubsystem.rawMoveHorizontal(-ArmConstants.horizBangBangSpeed);
    return withinMargin(this.armSubsystem.getHorizontalEncoder(), targetDist, ArmConstants.horizontalPIDTolerance);
  }

  private boolean runRot(double targetRot) {
    if(this.armSubsystem.getRotationalEncoder() < targetRot) this.armSubsystem.rawRotateGrabber(ArmConstants.rotBangBangSpeed);
    if(this.armSubsystem.getRotationalEncoder() > targetRot) this.armSubsystem.rawRotateGrabber(-ArmConstants.rotBangBangSpeed);
    return withinMargin(this.armSubsystem.getRotationalEncoder(), targetRot, ArmConstants.rotationalPIDTolerance);
  }

  private boolean withinMargin(double pos, double target, double margin) {
    return Math.abs(target - pos) <= margin;
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
