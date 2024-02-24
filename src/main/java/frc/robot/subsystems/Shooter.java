package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Shooter extends SubsystemBase {

    private CANSparkMax LMotor;
    private CANSparkMax RMotor;

    public Shooter() {

        // Assumption of use of a NEO brushless motor
        LMotor = new CANSparkMax(Settings.LCLIMBER_MOTOR_ID, MotorType.kBrushless);
        RMotor = new CANSparkMax(Settings.RCLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Restore factory defaults
        LMotor.restoreFactoryDefaults();
        RMotor.restoreFactoryDefaults();

        // Set the current limits
        LMotor.setSmartCurrentLimit(25);
        LMotor.setSecondaryCurrentLimit(30);
        RMotor.setSmartCurrentLimit(25);
        RMotor.setSecondaryCurrentLimit(30);

        // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // break the robot!
        LMotor.setOpenLoopRampRate(0.2);
        RMotor.setOpenLoopRampRate(0.2);

        // Set the idle mode to brake
        LMotor.setIdleMode(IdleMode.kBrake);
        RMotor.setIdleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        LMotor.setCANTimeout(20);
        RMotor.setCANTimeout(20);

        // Update the settings
        LMotor.burnFlash();
        RMotor.burnFlash();
    }

    public void intake() {
        LMotor.set(Settings.INTAKE_SPEED);
        RMotor.set(Settings.INTAKE_SPEED);
    }

    public void outtake(boolean fast) {
        if (fast) {
            LMotor.set(Settings.FAST_OUTTAKE_SPEED);
            RMotor.set(Settings.FAST_OUTTAKE_SPEED);
        } else {
            LMotor.set(Settings.SLOW_OUTTAKE_SPEED);
            RMotor.set(Settings.SLOW_OUTTAKE_SPEED);
        }
    }

    public void stop() {
        LMotor.set(0);
        RMotor.set(0);
    }

}
