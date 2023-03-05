package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax verticalMotor;
    private final CANSparkMax horizontalMotor;
    private final CANSparkMax intakeMotor;

    private final DoubleSolenoid extensionRetractionSolenoid;
    private final DoubleSolenoid openCloseSolenoid;

    private final Solenoid armBrake;

    private final DigitalInput armAtVericalLimit;

    private final double speedScaleFactor = 1.2;
    private final double intakespeedScaleFactor = 0.2;

    private double lastVertTeleopPos = 0;

    private double maxPIDOutput = 1;
    private double currentHeightTarget = 0;
    private double vMargin = 0.4;
    private double currentDistTarget = 0;
    private double hMargin = 0.4;
    private double currentIntakeTarget = 0;
    private double rMargin = 0.4;
    
    private Value grabberState = Value.kReverse;
    private Value extensionState = Value.kReverse;

    private final PIDController m_armVerticalPositionController = new PIDController(
            ArmConstants.kPArmVerticalController,
            ArmConstants.kIArmVerticalController,
            ArmConstants.kDArmVerticalController);

    private final PIDController m_armHorizontalPositionController = new PIDController(
            ArmConstants.kPArmHorizontalController,
            ArmConstants.kIArmHorizontalController,
            ArmConstants.kDArmHorizontalController);

    private final PIDController m_armIntakeController = new PIDController(
            ArmConstants.kPArmIntakeController,
            ArmConstants.kIArmIntakeController,
            ArmConstants.kDArmIntakeController);

    public ArmSubsystem() {
        this.verticalMotor = new CANSparkMax(ArmConstants.verticalMotor, MotorType.kBrushless);
        this.verticalMotor.setInverted(true);
        this.horizontalMotor = new CANSparkMax(ArmConstants.horizontalMotor, MotorType.kBrushless);
        this.intakeMotor = new CANSparkMax(ArmConstants.intakeMotor, MotorType.kBrushless);

        this.armAtVericalLimit = new DigitalInput(ArmConstants.upperArmLimit);

        this.extensionRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                ArmConstants.extensionSolenoid, ArmConstants.retractionSolenoid);

        this.openCloseSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.openSolenoid,
                ArmConstants.closeSolenoid);

        this.armBrake = new Solenoid(PneumaticsModuleType.CTREPCM, ArmConstants.brakeSolenoid);

        this.m_armHorizontalPositionController.setTolerance(ArmConstants.horizontalPIDTolerance);
        this.m_armVerticalPositionController.setTolerance(ArmConstants.verticalPIDTolerance);
        this.m_armIntakeController.setTolerance(ArmConstants.intakePIDTolerance);
    }

    @Override
    public void periodic() {
        this.openCloseSolenoid.set(this.grabberState);
        this.extensionRetractionSolenoid.set(this.extensionState);
    }

    public void moveVertical(double speed) {
        currentHeightTarget += speed * speedScaleFactor;
        this.setLastVertTeleopPos(currentHeightTarget);
        runVertical();
    }

    public boolean goToVertical(double target) {
        currentHeightTarget = Double.isNaN(target) ? currentHeightTarget : target;
        this.setLastVertTeleopPos(currentHeightTarget);
        runVertical();
        return m_armVerticalPositionController.atSetpoint();
    }

    public void holdVertical() {
        currentHeightTarget = lastVertTeleopPos;
        runVertical();
    }

    public boolean goToHorizontal(double target) {
        currentDistTarget = Double.isNaN(target) ? currentDistTarget : target;
        runHorizontal();
        return m_armHorizontalPositionController.atSetpoint();
    }

    public void moveHorizontal(double speed) {
        currentDistTarget += speed * speedScaleFactor;
        ;
        runHorizontal();
    }

    public void holdHorizontal() {
        runHorizontal();
    }

    public boolean goToIntake(double target) {
        currentIntakeTarget = Double.isNaN(target) ? currentIntakeTarget : target;
        runIntake();
        return m_armIntakeController.atSetpoint();
    }

    public void intakeGrabber(double speed) {
        currentIntakeTarget += speed * this.intakespeedScaleFactor;
        runIntake();
    }

    public void holdIntake() {
        runIntake();
    }

    public void holdAll() {
        this.holdHorizontal();
        this.holdIntake();
        this.holdVertical();
    }

    private void runVertical() {
        double targetSpeed = clampedVerticalCalculate();
        this.rawMoveVertical(targetSpeed);

    }

    private void runHorizontal() {
        double targetSpeed = clampedHorizontalCalculate();
        this.rawMoveHorizontal(targetSpeed);
    }

    private void runIntake() {
        double targetSpeed = clampedIntakeCalculate();
        this.rawIntakeGrabber(targetSpeed);
    }

    public void teleopMoveVertical(double speed) {
        if (Math.abs(speed) > 0) {
            this.rawMoveVertical(speed);
            this.setLastVertTeleopPos(this.getVertEncoder());
        } else
            this.holdVertical();
    }

    public void rawMoveVertical(double speed) {
        this.verticalMotor.set(speed);
        if (Math.abs(speed) > 0)
            this.armBrake.set(true);
        else
            this.armBrake.set(false);
    }

    public void rawMoveHorizontal(double speed) {
        this.horizontalMotor.set(speed);
    }

    public void rawIntakeGrabber(double speed) {
        this.intakeMotor.set(speed);
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
        double set = this.m_armIntakeController.getSetpoint();
        double at = this.intakeMotor.getEncoder().getPosition();
        return Math.abs(set - at) <= this.rMargin;
    }

    public void extend() {
        this.extensionState = Value.kForward;
    }

    public void retract() {
        this.extensionState = Value.kReverse;
    }

    public void open() {
        this.grabberState = Value.kForward;
    }

    public void close() {
        this.grabberState = Value.kReverse;
    }

    public void setMaxArmSpeeds(double maxSpeed) {
        this.maxPIDOutput = maxSpeed;
    }

    public void stop() {
        this.verticalMotor.stopMotor();
        this.horizontalMotor.stopMotor();
        this.intakeMotor.stopMotor();
        this.armBrake.set(false);
    }

    private double clampedVerticalCalculate() {
        return MathUtil.clamp(m_armVerticalPositionController.calculate(this.verticalMotor.getEncoder().getPosition(),
                currentHeightTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

    private double clampedHorizontalCalculate() {
        return MathUtil.clamp(m_armVerticalPositionController.calculate(this.horizontalMotor.getEncoder().getPosition(),
                currentDistTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

    private double clampedIntakeCalculate() {
        return MathUtil.clamp(this.m_armIntakeController.calculate(this.intakeMotor.getEncoder().getPosition(),
                this.currentIntakeTarget), -this.maxPIDOutput, this.maxPIDOutput);
    }

    public void setLastVertTeleopPos(double pos) {
        this.lastVertTeleopPos = pos;
    }

    public double getVertEncoder() {
        return this.verticalMotor.getEncoder().getPosition();
    }

    public double getHorizontalEncoder() {
        return this.horizontalMotor.getEncoder().getPosition();
    }

    public double getIntakeEncoder() {
        return this.intakeMotor.getEncoder().getPosition();
    }

    public boolean atVerticalLimit() {
        return armAtVericalLimit.get();
    }
}