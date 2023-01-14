package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final WPI_TalonFX m_driveMotor;
    private final CANSparkMax m_turningMotor;
  
    private final WPI_CANCoder m_turningEncoder;
  
    private final PIDController m_drivePIDController =
        new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
  

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID) {
        this.m_driveMotor = new WPI_TalonFX(driveMotorID);

        this.m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        this.m_turningEncoder = new WPI_CANCoder(turningEncoderID);

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningEncoder.getAbsolutePosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveMotor.getSelectedSensorPosition(), new Rotation2d(m_turningEncoder.getAbsolutePosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getAbsolutePosition()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
            m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);

        final double turnOutput = 
            m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(), state.angle.getRadians());

        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }
}
