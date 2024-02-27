package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class ArmShooter extends SubsystemBase {

    private CANSparkMax sMotor;

    public ArmShooter() {

        sMotor = new CANSparkMax(Settings.SHOOTER_MOTOR_ID, MotorType.kBrushless);

        sMotor.restoreFactoryDefaults();

        sMotor.setSmartCurrentLimit(80, 80);
        sMotor.setSecondaryCurrentLimit(85);

        sMotor.setOpenLoopRampRate(0.2);

        sMotor.setCANTimeout(20);

        sMotor.burnFlash();
    }

    public void sIntake() {
        sMotor.set(Settings.sINTAKE_SPEED);
    }

    public void sOuttake() {
        sMotor.set(Settings.sOUTTAKE_SPEED);
    }

    public void sStop() {
        sMotor.set(0);
    }
}
