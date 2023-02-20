package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final WPI_TalonFX m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final WPI_CANCoder m_turningEncoder;

    private final PIDController m_drivePIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kPModuleTurningController, 0,
            0);

    public SwerveModule(int driveMotorID, int turningMotorID, int turningEncoderID) {

        this.m_driveMotor = new WPI_TalonFX(driveMotorID);

        this.m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        this.m_turningMotor.setInverted(true);
        this.m_turningEncoder = new WPI_CANCoder(turningEncoderID);
        this.m_turningEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        m_turningPIDController.enableContinuousInput(-180, 180);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(), Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(), Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));
    }

    private double getRotationsPerSecond() {
        return (m_driveMotor.getSelectedSensorVelocity() * 10) / (ModuleConstants.driveEncoderTicksPerRotation * ModuleConstants.gearRatio); // Data
                                                                                                               // is
                                                                                                               // returned
                                                                                                               // in
                                                                                                               // ticks
                                                                                                               // per
                                                                                                               // 100ms,
                                                                                                               // multiply
                                                                                                               // by 10
                                                                                                               // to get
                                                                                                               // ticks/second,
                                                                                                               // then
                                                                                                               // divide
                                                                                                               // by
                                                                                                               // ticks
                                                                                                               // per
                                                                                                               // rotation
                                                                                                               // for
                                                                                                               // rotations/second
    }

    private double getRotationsFromTicks(double ticks) {
        return ticks / (ModuleConstants.driveEncoderTicksPerRotation * ModuleConstants.gearRatio);
    }

    private double getDriveVelocity() {
        return (getRotationsPerSecond() * ModuleConstants.metersPerRotation);
    }

    private double getDrivePosition() {
        return getRotationsFromTicks(m_driveMotor.getSelectedSensorPosition()) / ModuleConstants.metersPerRotation;
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                Rotation2d.fromDegrees(m_turningEncoder.getAbsolutePosition()));

        // Calculate the drive output from the drive PID controller.

        double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

        final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(),
                state.angle.getDegrees());
        
        m_driveMotor.set(driveOutput);
        m_turningMotor.set(turnOutput);
    }

    public void stop() {
        m_driveMotor.stopMotor();
        m_turningMotor.stopMotor();
    }

   // public void stop() {
     //   m_driveMotor.stopMotor();
       // m_turningMotor.stopMotor();
   // }

}
