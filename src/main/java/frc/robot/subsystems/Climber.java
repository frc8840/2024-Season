package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Settings;
import frc.team_8840_lib.info.console.Logger;

public class Climber extends SubsystemBase {

    private CANSparkMax lMotor;
    private CANSparkMax rMotor;

    public RelativeEncoder lEncoder;
    public RelativeEncoder rEncoder;

    private final SparkPIDController lController;
    private final SparkPIDController rController;

    public Climber() {

        // Assumption of use of a NEO brushless motor
        lMotor = new CANSparkMax(Settings.LCLIMBER_MOTOR_ID, MotorType.kBrushless);
        lEncoder = lMotor.getEncoder();
        lController = lMotor.getPIDController();

        rMotor = new CANSparkMax(Settings.RCLIMBER_MOTOR_ID, MotorType.kBrushless);
        rEncoder = rMotor.getEncoder();
        rController = rMotor.getPIDController();

        // Restore factory defaults
        lMotor.restoreFactoryDefaults();
        rMotor.restoreFactoryDefaults();

        // Set the current limits
        lMotor.setSmartCurrentLimit(80, 80);
        lMotor.setSecondaryCurrentLimit(85);
        rMotor.setSmartCurrentLimit(80, 80);
        rMotor.setSecondaryCurrentLimit(85);

        // // Set the ramp rate since it jumps to full speed too quickly - don't want to
        // // break the robot!
        lMotor.setOpenLoopRampRate(0.2);
        rMotor.setOpenLoopRampRate(0.2);

        // Set the idle mode to brake
        lMotor.setIdleMode(IdleMode.kBrake);
        rMotor.setIdleMode(IdleMode.kBrake);

        // Set the CAN timeout to 20ms
        lMotor.setCANTimeout(20);
        rMotor.setCANTimeout(20);

        lMotor.enableVoltageCompensation(12.0);
        rMotor.enableVoltageCompensation(12.0);

        // controller and encoder
        lController.setP(Constants.Climber.climberKP);
        lController.setI(Constants.Climber.climberKI);
        lController.setD(Constants.Climber.climberKD);
        lController.setFF(Constants.Climber.climberKFF);

        rController.setP(Constants.Climber.climberKP);
        rController.setI(Constants.Climber.climberKI);
        rController.setD(Constants.Climber.climberKD);
        rController.setFF(Constants.Climber.climberKFF);

        lEncoder.setPosition(0.0);
        rEncoder.setPosition(0.0);

        // Update the settings
        lMotor.burnFlash();
        rMotor.burnFlash();
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Logger.Log("interrupted");
        }
        Logger.Log("L position: " + lEncoder.getPosition());
        Logger.Log("R position: " + rEncoder.getPosition());
    }

    public void Lintake() {
        lMotor.set(Settings.INTAKE_SPEED);
        Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Rintake() {
        rMotor.set(Settings.INTAKE_SPEED);
        Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void Louttake() {
        lMotor.set(Settings.OUTTAKE_SPEED);
        Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Routtake() {
        rMotor.set(Settings.OUTTAKE_SPEED);
        Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void leftStop() {
        lMotor.set(0);
    }

    public void rightStop() {
        rMotor.set(0);
    }

    public void climb() {
        lController.setReference(10, CANSparkMax.ControlType.kPosition);
        rController.setReference(10, CANSparkMax.ControlType.kPosition);
    }

    public void drop() {
        lController.setReference(0, CANSparkMax.ControlType.kPosition);
        rController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

}
