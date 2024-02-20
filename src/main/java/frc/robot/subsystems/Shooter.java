package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Shooter extends SubsystemBase {

    private CANSparkMax motor;

    public Shooter() {

        // Assumption of use of a NEO brushless motor
        motor = new CANSparkMax(Settings.ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Restore factory defaults
        motor.restoreFactoryDefaults();

        // Set the current limits
        motor.setSmartCurrentLimit(25);
        motor.setSecondaryCurrentLimit(30);

        // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // break the robot!
        motor.setOpenLoopRampRate(0.2);

        // Set the idle mode to brake
        motor.setIdleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        motor.setCANTimeout(20);

        // Update the settings
        motor.burnFlash();
    }

    public void intake() {
        motor.set(Settings.INTAKE_SPEED);
    }

    public void outtake(boolean fast) {
        if (fast) {
            motor.set(Settings.FAST_OUTTAKE_SPEED);
        } else {
            motor.set(Settings.SLOW_OUTTAKE_SPEED);
        }
    }

    public void stop() {
        motor.set(0);
    }

}
