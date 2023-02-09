package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private final double speedScaleFactor = 0.1;
    private final double rotationalspeedScaleFactor = 0.1;

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
        this.verticalMotor.set(speed * this.speedScaleFactor);
    }

    public void moveHorizontal(double speed) {
        this.horizontalMotor.set(speed * this.speedScaleFactor);
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