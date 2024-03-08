package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class ArmShooter extends SubsystemBase {

    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;

    public ArmShooter() {

        leftMotor = new CANSparkMax(Settings.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Settings.SHOOTER_MOTOR_ID2, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);

        leftMotor.setSmartCurrentLimit(100, 80);
        leftMotor.setSecondaryCurrentLimit(105);

        rightMotor.setSmartCurrentLimit(100, 80);
        rightMotor.setSecondaryCurrentLimit(105);

        // sMotor.setOpenLoopRampRate(0.2);
        // sMotor2.setOpenLoopRampRate(0.2);

        leftMotor.setCANTimeout(20);
        rightMotor.setCANTimeout(20);

        leftMotor.burnFlash();
        rightMotor.burnFlash();
    }

    public void shoot() {
        leftMotor.set(Settings.SHOOTER_OUT_SPEED);
        rightMotor.set(-Settings.SHOOTER_OUT_SPEED);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public void gethard() {
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
}
