// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    
    public static final int kFrontLeftDriveID = 21;
    public static final int kRearLeftDriveID = 31;
    public static final int kFrontRightDriveID = 11;
    public static final int kRearRightDriveID = 41;

    public static final int kFrontLeftTurningID = 22;
    public static final int kRearLeftTurningID = 32;
    public static final int kFrontRightTurningID = 12;
    public static final int kRearRightTurningID = 41;

    public static final int kFrontLeftTurnEncoderID = 23;
    public static final int kRearLeftTurnEncoderID = 31;
    public static final int kFrontRightTurnEncoderID = 13;
    public static final int kRearRightTurnEncoderID = 43;

    public static final double kTrackWidth = 24 + (3/8); // 24 and 3/8in
    // Distance between centers of right and left wheels on robot

    public static final double kWheelBase = 24 + (3/8); // 24 and 3/8in
    // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    // public static final double ksVolts = "Fix";
    // public static final double kvVoltSecondsPerMeter = "Fix";
    // public static final double kaVoltSecondsSquaredPerMeter = "Fix";

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static class ModuleConstants {
    public static final double kPModuleDriveController = 1;
    public static final double kPModuleTurningController = 1;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
