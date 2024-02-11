package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Roller extends SubsystemBase {

    private CANSparkMax rollerMotor;

    public Roller() {

        // Assumption of use of a NEO brushless motor
        rollerMotor = new CANSparkMax(Settings.ROLLER_MOTOR_ID, MotorType.kBrushless);

        // Restore factory defaults
        rollerMotor.restoreFactoryDefaults();

        // Set the current limits
        rollerMotor.setSmartCurrentLimit(25);
        rollerMotor.setSecondaryCurrentLimit(30);

        // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // break the robot!
        rollerMotor.setOpenLoopRampRate(0.2);

        // Set the idle mode to brake
        rollerMotor.setIdleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        rollerMotor.setCANTimeout(20);

        // Update the settings
        rollerMotor.burnFlash();
    }

    public void intake() {
        rollerMotor.set(Settings.INTAKE_SPEED);
    }

    public void outtake(boolean fast) {
        if (fast) {
            rollerMotor.set(Settings.FAST_OUTTAKE_SPEED);
        } else {
            rollerMotor.set(Settings.SLOW_OUTTAKE_SPEED);
        }
    }

    public void stop() {
        rollerMotor.set(0);
    }

}
