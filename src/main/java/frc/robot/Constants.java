// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class DriveConstants {

    private static final boolean False = false;

    public static boolean useVisionBasedOdemetryEstimation = false;

    public static final int kFrontRightDriveID = 11;
    public static final int kFrontLeftDriveID = 21;
    public static final int kRearLeftDriveID = 31;
    public static final int kRearRightDriveID = 41;

    public static final int kFrontRightTurningID = 12;
    public static final int kFrontLeftTurningID = 22;
    public static final int kRearLeftTurningID = 32;
    public static final int kRearRightTurningID = 42;

    public static final int kFrontRightTurnEncoderID = 13;
    public static final int kFrontLeftTurnEncoderID = 23;
    public static final int kRearLeftTurnEncoderID = 33;
    public static final int kRearRightTurnEncoderID = 43;

    public static final double kTrackWidth = 24 + (3 / 8); // 24 and 3/8in
    // Distance between centers of right and left wheels on robot

    public static final double kWheelBase = 24 + (3 / 8); // 24 and 3/8in
    // Distance between front and back wheels on robot

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 30;

    public static final double angleOffset = 90;
  }

  public static class ModuleConstants {
    public static final double kPModuleDriveController = 0.1;
    public static final double kPModuleTurningController = 0.01;
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;
    public static final int driveEncoderTicksPerMotorRotation = 2048; // 13824;
    public static final double gearRatio = 6.75;
    public static final double distanceScaleForOdemetry = 9.8868;
    public static final double driveEncoderTicksPerWheelRotation = driveEncoderTicksPerMotorRotation * gearRatio;
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);// 0.1016;
    public static final double metersPerRotation = (wheelDiameterMeters * Math.PI);
    public static final double metersPerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts, will fix this
        // later
        metersPerRotation / ((double) driveEncoderTicksPerMotorRotation * gearRatio);
  }

  public static class ArmConstants {
    public static final int verticalMotor = 51;
    public static final int horizontalMotor = 52;
    public static final int rotationalMotor = 53;

    public static final int extensionSolenoid = 2;
    public static final int retractionSolenoid = 1;
    public static final int openSolenoid = 0;
    public static final int closeSolenoid = 3;
    public static final int brakeSolenoid = 4;

    public static final int upperArmLimit = 0;

    public static final double kPArmVerticalController = 0.2;
    public static final double kIArmVerticalController = 0;
    public static final double kPArmHorizontalController = 0.1;
    public static final double kIArmHorizontalController = 0;
    public static final double kPArmRotationalController = 0.1;

    public static final double verticalRange = 43.7;
    public static final double horizontalRange = 63.86;
    public static final double horizontalStart = -9.6;
    public static final double rotationalRange = 3.1; // per direction

    public static final double horizontalPIDTolerance = 0.1;
    public static final double verticalPIDTolerance = 0.1;
    public static final double rotationalPIDTolerance = 0.1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class FieldConstants {
    static final double length = Units.feetToMeters(54);
    static final double width = Units.feetToMeters(27);
  }

  
  public static final class AutoConstants {
    public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(20, 10
    )
        .setKinematics(DriveConstants.kDriveKinematics);
    public static PIDController xController = new PIDController(10, 0, 0);
    public static PIDController yController = new PIDController(17, 0, 0);
    public static ProfiledPIDController thetaController = new ProfiledPIDController(
        0.75,
        0,
        0,
        new TrapezoidProfile.Constraints(ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
            ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
    public static double NO_MOVEMENT = Double.NaN;
    public static double MAX_ARM_SPEED_OUTPUT = 0.4;

    public static double unitsPerMeter = 4;
  }

  public static class VisionConstants {
    public static final Transform3d robotToCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(-4.75), Units.inchesToMeters(2.75), Units.inchesToMeters(13)),
        new Rotation3d(0, 0, 0));
        
    public static final String cameraName = "PSI_Cam";
  }
}
