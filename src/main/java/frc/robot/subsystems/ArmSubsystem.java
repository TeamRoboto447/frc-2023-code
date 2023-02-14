package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax verticalMotor;
    private final CANSparkMax horizontalMotor;
    private final CANSparkMax rotationalMotor;

    private final DoubleSolenoid extensionRetractionSolenoid;
    private final DoubleSolenoid openCloseSolenoid;

    private final double speedScaleFactor = 0.5;
    private final double rotationalspeedScaleFactor = 0.4;

    private double currentHeightTarget = 0;
    private double currentDistTarget = 0;
    private final PIDController m_armVerticalPositionController = new PIDController(ArmConstants.kPArmPositionController, 0, 0);
    private final PIDController m_armHorizontalPositionController = new PIDController(ArmConstants.kPArmPositionController, 0, 0);

    public ArmSubsystem() {
        this.verticalMotor = new CANSparkMax(ArmConstants.verticalMotor, MotorType.kBrushless);
        this.horizontalMotor = new CANSparkMax(ArmConstants.horizontalMotor, MotorType.kBrushless);
        this.rotationalMotor = new CANSparkMax(ArmConstants.rotationalMotor, MotorType.kBrushless);

        this.extensionRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                ArmConstants.extensionSolenoid, ArmConstants.retractionSolenoid);

        this.openCloseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.openSolenoid,
                ArmConstants.closeSolenoid);
    }

    public void moveVertical(double speed) {
        currentHeightTarget += speed;
        double targetSpeed = m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget);
        this.verticalMotor.set(targetSpeed);
    }

    public boolean goToVertical(double target) {
        currentHeightTarget = target;
        double targetSpeed = m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget);
        this.verticalMotor.set(targetSpeed);
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
        return m_armHorizontalPositionController.atSetpoint();
    }

    public void moveHorizontal(double speed) {
        currentDistTarget += speed;
        double targetSpeed = m_armHorizontalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget);
        this.horizontalMotor.set(targetSpeed);
    }

    public void holdHorizontal() {
        double targetSpeed = m_armVerticalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget);
        this.horizontalMotor.set(targetSpeed);
    }

    public void rotateGrabber(double speed) {
        this.rotationalMotor.set(speed * this.rotationalspeedScaleFactor);
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