package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax verticalMotor;
    private final CANSparkMax horizontalMotor;
    private final CANSparkMax rotationalMotor;

    private final DoubleSolenoid extensionRetractionSolenoid;
    private final DoubleSolenoid openCloseSolenoid;

    private final Solenoid armBrake;

    private final DigitalInput armAtVericalLimit;

    private final double speedScaleFactor = 1.2;
    private final double rotationalspeedScaleFactor = 0.2;

    private double maxPIDOutput = 1;
    private double currentHeightTarget = 0;
    private double vMargin = 0.4;
    private double currentDistTarget = 0;
    private double hMargin = 0.4;
    private double currentRotTarget = 0;
    private double rMargin = 0.4;
    private final PIDController m_armVerticalPositionController = new PIDController(ArmConstants.kPArmVerticalController, ArmConstants.kIArmVerticalController, 0);
    private final PIDController m_armHorizontalPositionController = new PIDController(ArmConstants.kPArmHorizontalController, ArmConstants.kIArmHorizontalController, 0);
    private final PIDController m_armRotationalController = new PIDController(ArmConstants.kPArmRotationalController, 0, 0);

    public ArmSubsystem() {
        this.verticalMotor = new CANSparkMax(ArmConstants.verticalMotor, MotorType.kBrushless);
        this.verticalMotor.setInverted(true);
        this.horizontalMotor = new CANSparkMax(ArmConstants.horizontalMotor, MotorType.kBrushless);
        this.rotationalMotor = new CANSparkMax(ArmConstants.rotationalMotor, MotorType.kBrushless);

        this.armAtVericalLimit = new DigitalInput(ArmConstants.upperArmLimit);

        this.extensionRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                ArmConstants.extensionSolenoid, ArmConstants.retractionSolenoid);

        this.openCloseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.openSolenoid,
                ArmConstants.closeSolenoid);

        this.armBrake = new Solenoid(PneumaticsModuleType.CTREPCM, ArmConstants.brakeSolenoid);

        this.m_armHorizontalPositionController.setTolerance(ArmConstants.horizontalPIDTolerance);
        this.m_armVerticalPositionController.setTolerance(ArmConstants.verticalPIDTolerance);
        this.m_armRotationalController.setTolerance(ArmConstants.rotationalPIDTolerance);
    }

    public void moveVertical(double speed) {
        currentHeightTarget += speed * speedScaleFactor;
        runVertical();
    }

    public boolean goToVertical(double target) {
        currentHeightTarget = Double.isNaN(target) ? currentHeightTarget : target;
        runVertical();
        return m_armVerticalPositionController.atSetpoint();
    }

    public void holdVertical() {
        runVertical();
    }

    public boolean goToHorizontal(double target) {
        currentDistTarget = Double.isNaN(target) ? currentDistTarget : target;
        runHorizontal();
        return m_armHorizontalPositionController.atSetpoint();
    }

    public void moveHorizontal(double speed) {
        currentDistTarget += speed * speedScaleFactor;;
        runHorizontal();
    }

    public void holdHorizontal() {
        runHorizontal();
    }

    public boolean goToRotation(double target) {
        currentRotTarget = Double.isNaN(target) ? currentRotTarget : target;
        runRotation();
        return m_armRotationalController.atSetpoint();
    }

    public void rotateGrabber(double speed) {
        currentRotTarget += speed * this.rotationalspeedScaleFactor;
        runRotation();
    }

    public void holdRotation() {
        runRotation();
    }

    public void holdAll() {
        this.holdHorizontal();
        this.holdRotation();
        this.holdVertical();
    }

    private void runVertical() {
        double targetSpeed = clampedVerticalCalculate();
        this.rawMoveVertical(targetSpeed);
        SmartDashboard.putNumber("Vertical Encoder Value", this.verticalMotor.getEncoder().getPosition());

    }

    private void runHorizontal() {
        double targetSpeed = clampedHorizontalCalculate();
        this.rawMoveHorizontal(targetSpeed);
        SmartDashboard.putNumber("Horizontal Encoder Value", this.horizontalMotor.getEncoder().getPosition());
    }

    private void runRotation() {
        double targetSpeed = clampedRotationalCalculate();
        this.rawRotateGrabber(targetSpeed);
        SmartDashboard.putNumber("Rotational Encoder Value", this.rotationalMotor.getEncoder().getPosition());
    }

    public void rawMoveVertical(double speed) {
        this.verticalMotor.set(speed);
       if(Math.abs(speed)>0)
        this.armBrake.set(true);
       else
        this.armBrake.set(false);
    }
    
    public void rawMoveHorizontal(double speed) {
        this.horizontalMotor.set(speed);
    }
    
    public void rawRotateGrabber(double speed) {
        this.rotationalMotor.set(speed);
    }

    public boolean withinMarginV() {
        double set = this.m_armVerticalPositionController.getSetpoint();
        double at = this.verticalMotor.getEncoder().getPosition();
        return Math.abs(set - at) <= this.vMargin;
    }

    public boolean withinMarginH() {
        double set = this.m_armHorizontalPositionController.getSetpoint();
        double at = this.horizontalMotor.getEncoder().getPosition();
        return Math.abs(set - at) <= this.hMargin;
    }

    public boolean withinMarginR() {
        double set = this.m_armRotationalController.getSetpoint();
        double at = this.rotationalMotor.getEncoder().getPosition();
        return Math.abs(set - at) <= this.rMargin;
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

    public void setMaxArmSpeeds(double maxSpeed) {
        this.maxPIDOutput = maxSpeed;
    }

    public void stop() {
        this.verticalMotor.stopMotor();
        this.horizontalMotor.stopMotor();
        this.rotationalMotor.stopMotor();
        this.armBrake.set(false);
    }

    private double clampedVerticalCalculate() {
        return MathUtil.clamp(m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(), currentHeightTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

    private double clampedHorizontalCalculate() {
        return MathUtil.clamp(m_armVerticalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(), currentDistTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

    private double clampedRotationalCalculate() {
        return MathUtil.clamp(this.m_armRotationalController.calculate(this.rotationalMotor.getEncoder().getPosition(), this.currentRotTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

}