package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxAcc, allowedErr;
        kP = 0.1;
        kI = 0.0;
        kD = 5.0;
        kIz = 0.0;
        kFF = 0.0;
        kMaxOutput = 1;
        kMinOutput = -1;

        minVel = 0; // rpm // TODO is this correct?
        maxVel = 2500; // rpm
        maxAcc = 1500;
        allowedErr = 0; // TODO is this correct?

        int smartMotionSlot = 0;

        // left
        lController.setP(kP);
        lController.setI(kI);
        lController.setD(kD);
        lController.setIZone(kIz);
        lController.setFF(kFF);
        // lController.setOutputRange(kMinOutput, kMaxOutput);
        lController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        // lController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // lController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        // lController.setSmartMotionAllowedClosedLoopError(allowedErr,
        // smartMotionSlot);
        // right
        rController.setP(kP);
        rController.setI(kI);
        rController.setD(kD);
        rController.setIZone(kIz);
        rController.setFF(kFF);
        // rController.setOutputRange(kMinOutput, kMaxOutput);
        rController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        // rController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        // rController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        // rController.setSmartMotionAllowedClosedLoopError(allowedErr,
        // smartMotionSlot);

        lEncoder.setPosition(0.0);
        rEncoder.setPosition(0.0);

        // Update the settings
        lMotor.burnFlash();
        rMotor.burnFlash();
        Logger.Log("L position: " + lEncoder.getPosition());
        Logger.Log("R position: " + rEncoder.getPosition());
    }

    public void Lintake() {
        lMotor.set(Settings.CLIMBER_INTAKE_SPEED);
        Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Rintake() {
        rMotor.set(Settings.CLIMBER_INTAKE_SPEED);
        Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void Louttake() {
        lMotor.set(Settings.CLIMBER_OUTTAKE_SPEED);
        Logger.Log("lMotor current: " + lMotor.getOutputCurrent());
    }

    public void Routtake() {
        rMotor.set(Settings.CLIMBER_OUTTAKE_SPEED);
        Logger.Log("rMotor current: " + lMotor.getOutputCurrent());
    }

    public void leftStop() {
        lMotor.set(0);
        lMotor.setIdleMode(IdleMode.kBrake);
    }

    public void rightStop() {
        rMotor.set(0);
        rMotor.setIdleMode(IdleMode.kBrake);
    }

    public void climb() {
        lController.setReference(30, CANSparkMax.ControlType.kPosition);
        rController.setReference(30, CANSparkMax.ControlType.kPosition);
    }

    public void drop() {
        lController.setReference(0, CANSparkMax.ControlType.kPosition);
        rController.setReference(0, CANSparkMax.ControlType.kPosition);
    }

}
