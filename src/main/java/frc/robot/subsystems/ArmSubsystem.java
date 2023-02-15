package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax verticalMotor;
    private final CANSparkMax horizontalMotor;
    private final CANSparkMax rotationalMotor;

    private final DoubleSolenoid extensionRetractionSolenoid;
    private final DoubleSolenoid openCloseSolenoid;

    private final double speedScaleFactor = 1.2;
    private final double rotationalspeedScaleFactor = 0.2;

    private double currentHeightTarget = 0;
    private double currentDistTarget = 0;
    private double currentRotTarget = 0;
    private final PIDController m_armVerticalPositionController = new PIDController(ArmConstants.kPArmVerticalController, ArmConstants.kIArmVerticalController, 0);
    private final PIDController m_armHorizontalPositionController = new PIDController(ArmConstants.kPArmHorizontalController, ArmConstants.kIArmHorizontalController, 0);
    private final PIDController m_armRotationalController = new PIDController(ArmConstants.kPArmRotationalController, 0, 0);

    public ArmSubsystem() {
        this.verticalMotor = new CANSparkMax(ArmConstants.verticalMotor, MotorType.kBrushless);
        this.horizontalMotor = new CANSparkMax(ArmConstants.horizontalMotor, MotorType.kBrushless);
        this.rotationalMotor = new CANSparkMax(ArmConstants.rotationalMotor, MotorType.kBrushless);

        this.extensionRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                ArmConstants.extensionSolenoid, ArmConstants.retractionSolenoid);

        this.openCloseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.openSolenoid,
                ArmConstants.closeSolenoid);

        this.m_armHorizontalPositionController.setTolerance(ArmConstants.horizontalPIDTolerance);
        this.m_armVerticalPositionController.setTolerance(ArmConstants.verticalPIDTolerance);
        this.m_armRotationalController.setTolerance(ArmConstants.rotationalPIDTolerance);
    }

    public void moveVertical(double speed) {
        currentHeightTarget += speed * speedScaleFactor;
        double targetSpeed = m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget);
        this.verticalMotor.set(targetSpeed);
        SmartDashboard.putNumber("Vertical Encoder Value", this.verticalMotor.getEncoder().getPosition());
    }

    public boolean goToVertical(double target) {
        currentHeightTarget = target;
        double targetSpeed = m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget);
        this.verticalMotor.set(targetSpeed);
        SmartDashboard.putNumber("Vertical Encoder Value", this.verticalMotor.getEncoder().getPosition());
        return m_armVerticalPositionController.atSetpoint();
    }

    public void holdVertical() {
        double targetSpeed = m_armHorizontalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget);
        this.verticalMotor.set(targetSpeed);
    }

    public boolean goToHorizontal(double target) {
        currentDistTarget = target;
        double targetSpeed = m_armHorizontalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget);
        this.horizontalMotor.set(targetSpeed);
        SmartDashboard.putNumber("Horizontal Encoder Value", this.horizontalMotor.getEncoder().getPosition());
        return m_armHorizontalPositionController.atSetpoint();
    }

    public void moveHorizontal(double speed) {
        currentDistTarget += speed * speedScaleFactor;;
        double targetSpeed = m_armHorizontalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget);
        this.horizontalMotor.set(targetSpeed);
        SmartDashboard.putNumber("Horizontal Encoder Value", this.horizontalMotor.getEncoder().getPosition());
    }

    public void holdHorizontal() {
        double targetSpeed = m_armVerticalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget);
        this.horizontalMotor.set(targetSpeed);
    }

    public boolean goToRotation(double target) {
        currentRotTarget = target;
        double targetSpeed = m_armRotationalController.calculate(this.rotationalMotor.getEncoder().getPosition(), currentRotTarget);
        this.rotationalMotor.set(targetSpeed);
        return m_armRotationalController.atSetpoint();
    }

    public void rotateGrabber(double speed) {
        currentRotTarget += speed * this.rotationalspeedScaleFactor;
        double targetSpeed = m_armRotationalController.calculate(this.rotationalMotor.getEncoder().getPosition(), currentRotTarget);
        this.rotationalMotor.set(targetSpeed);
        SmartDashboard.putNumber("Rotational Encoder Value", this.rotationalMotor.getEncoder().getPosition());
    }

    public void holdRotation() {
        double targetSpeed = m_armRotationalController.calculate(this.rotationalMotor.getEncoder().getPosition(), currentRotTarget);
        this.rotationalMotor.set(targetSpeed);
    }

    public void holdAll() {
        this.holdHorizontal();
        this.holdRotation();
        this.holdVertical();
    }

    public void extend() {
        this.extensionRetractionSolenoid.set(Value.kForward);
    }

    public void retract() {
        this.extensionRetractionSolenoid.set(Value.kReverse);
    }

    public void open() {
        this.openCloseSolenoid.set(Value.kForward);
    }

    public void close() {
        this.openCloseSolenoid.set(Value.kReverse);
    }

}