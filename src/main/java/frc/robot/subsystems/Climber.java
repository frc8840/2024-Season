package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;

public class Climber extends SubsystemBase {

    private CANSparkMax lMotor;
    private CANSparkMax rMotor;

    public Climber() {

        // Assumption of use of a NEO brushless motor
        lMotor = new CANSparkMax(Settings.LCLIMBER_MOTOR_ID, MotorType.kBrushless);
        rMotor = new CANSparkMax(Settings.RCLIMBER_MOTOR_ID, MotorType.kBrushless);

        // Restore factory defaults
        lMotor.restoreFactoryDefaults();
        rMotor.restoreFactoryDefaults();

        // Set the current limits
        lMotor.setSmartCurrentLimit(25);
        lMotor.setSecondaryCurrentLimit(30);
        rMotor.setSmartCurrentLimit(25);
        rMotor.setSecondaryCurrentLimit(30);

        // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // break the robot!
        lMotor.setOpenLoopRampRate(0.2);
        rMotor.setOpenLoopRampRate(0.2);

        // Set the idle mode to brake
        lMotor.setIdleMode(IdleMode.kBrake);
        rMotor.setIdleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        lMotor.setCANTimeout(20);
        rMotor.setCANTimeout(20);

        // Update the settings
        lMotor.burnFlash();
        rMotor.burnFlash();
    }

    public void Lintake() {
        lMotor.set(Settings.INTAKE_SPEED);
    }

    public void Rintake() {
        rMotor.set(Settings.INTAKE_SPEED);
    }

    public void Louttake() {
        lMotor.set(Settings.OUTTAKE_SPEED);
    }

    public void Routtake() {
        rMotor.set(Settings.OUTTAKE_SPEED);
    }

    public void leftStop() {
        lMotor.set(0);
    }

    public void rightStop() {
        rMotor.set(0);
    }

}
