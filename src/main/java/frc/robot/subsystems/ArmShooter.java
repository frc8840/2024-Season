package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class ArmShooter extends SubsystemBase {

    private CANSparkMax sMotor;
    private CANSparkMax sMotor2;

    public ArmShooter() {

        sMotor = new CANSparkMax(Settings.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        sMotor2 = new CANSparkMax(Settings.SHOOTER_MOTOR_ID2, MotorType.kBrushless);

        sMotor.restoreFactoryDefaults();
        sMotor2.restoreFactoryDefaults();

        sMotor.setSmartCurrentLimit(80, 80);
        sMotor.setSecondaryCurrentLimit(85);
        sMotor2.setSmartCurrentLimit(80, 80);
        sMotor2.setSecondaryCurrentLimit(85);

        sMotor.setOpenLoopRampRate(0.2);
        sMotor2.setOpenLoopRampRate(0.2);

        sMotor.setCANTimeout(20);
        sMotor2.setCANTimeout(20);

        sMotor.burnFlash();
        sMotor2.burnFlash();
    }

    public void sIntake() {
        sMotor.set(Settings.sINTAKE_SPEED);
        sMotor2.set(Settings.sINTAKE_SPEED);
    }

    public void sOuttake() {
        sMotor.set(Settings.sOUTTAKE_SPEED);
        sMotor2.set(Settings.sOUTTAKE_SPEED);
    }

    public void sStop() {
        sMotor.set(0);
        sMotor2.set(0);
    }
}
