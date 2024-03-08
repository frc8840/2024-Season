package frc.robot.subsystems;

import javax.swing.plaf.synth.SynthFormattedTextFieldUI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.team_8840_lib.info.console.Logger;

public class Arm extends SubsystemBase {
    private CANSparkMax shoulderMotor;
    private CANSparkMax elbowMotor;
    private CANSparkMax wristMotor;
    private RelativeEncoder shoulderEncoder;
    private RelativeEncoder elbowEncoder;
    private RelativeEncoder wristEncoder;
    private SparkPIDController shoulderPID;
    private SparkPIDController elbowPID;
    private SparkPIDController wristPID;
    private ArmPosition position = ArmPosition.REST;

    public Arm() {
        shoulderMotor = new CANSparkMax(Settings.SHOULDER_MOTOR_ID, MotorType.kBrushless);
        elbowMotor = new CANSparkMax(Settings.ELBOW_MOTOR_ID, MotorType.kBrushless);
        wristMotor = new CANSparkMax(Settings.WRIST_MOTOR_ID, MotorType.kBrushless);

        shoulderEncoder = shoulderMotor.getEncoder();
        elbowEncoder = elbowMotor.getEncoder();
        wristEncoder = wristMotor.getEncoder();

        shoulderEncoder.setPosition(0);
        elbowEncoder.setPosition(0);
        wristEncoder.setPosition(0);

        shoulderMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();
        wristMotor.restoreFactoryDefaults();

        shoulderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        wristMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        shoulderMotor.setSmartCurrentLimit(80);
        elbowMotor.setSmartCurrentLimit(80);
        wristMotor.setSmartCurrentLimit(80);

        shoulderMotor.setSecondaryCurrentLimit(85);
        elbowMotor.setSecondaryCurrentLimit(85);
        wristMotor.setSecondaryCurrentLimit(85);

        shoulderMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        elbowMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        wristMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);

        shoulderMotor.enableVoltageCompensation(12);
        elbowMotor.enableVoltageCompensation(12);
        wristMotor.enableVoltageCompensation(12);

        shoulderEncoder.setPositionConversionFactor((1 / Settings.SHOULDER_GEAR_RATIO) * 360);
        elbowEncoder.setPositionConversionFactor((1 / Settings.ELBOW_GEAR_RATIO) * 360);
        wristEncoder.setPositionConversionFactor((1 / Settings.WRIST_GEAR_RATIO) * 360);

        shoulderPID = shoulderMotor.getPIDController();
        elbowPID = elbowMotor.getPIDController();
        wristPID = wristMotor.getPIDController();

        shoulderPID.setP(Settings.SHOULDER_PID.kP);
        shoulderPID.setI(Settings.SHOULDER_PID.kI);
        shoulderPID.setD(Settings.SHOULDER_PID.kD);
        shoulderPID.setIZone(Settings.SHOULDER_PID.kIZone);
        shoulderPID.setFF(Settings.SHOULDER_PID.kF);

        elbowPID.setP(Settings.ELBOW_PID.kP);
        elbowPID.setI(Settings.ELBOW_PID.kI);
        elbowPID.setD(Settings.ELBOW_PID.kD);
        elbowPID.setIZone(Settings.ELBOW_PID.kIZone);
        elbowPID.setFF(Settings.ELBOW_PID.kF);

        wristPID.setP(Settings.WRIST_PID.kP);
        wristPID.setI(Settings.WRIST_PID.kI);
        wristPID.setD(Settings.WRIST_PID.kD);
        wristPID.setIZone(Settings.WRIST_PID.kIZone);
        wristPID.setFF(Settings.WRIST_PID.kF);

        shoulderPID.setOutputRange(-Settings.MAX_SHOULDER_SPEED, Settings.MAX_SHOULDER_SPEED);
        elbowPID.setOutputRange(-Settings.MAX_ELBOW_SPEED, Settings.MAX_ELBOW_SPEED);
        wristPID.setOutputRange(-Settings.MAX_WRIST_SPEED, Settings.MAX_WRIST_SPEED);

        shoulderMotor.burnFlash();
        elbowMotor.burnFlash();
        wristMotor.burnFlash();
    }

    public void setArmPosition(ArmPosition position) {
        this.position = position;

        // Logger.Log("shoulder position before:" + shoulderEncoder.getPosition());
        shoulderPID.setReference(
                position.shoulderAngle,
                ControlType.kPosition,
                0);

        elbowPID.setReference(
                position.elbowAngle,
                ControlType.kPosition,
                0);

        wristPID.setReference(
                position.wristAngle,
                ControlType.kPosition,
                0);
    }

    public void relax() {
        shoulderMotor.setIdleMode(IdleMode.kCoast);
        shoulderMotor.set(0);
        elbowMotor.setIdleMode(IdleMode.kCoast);
        elbowMotor.set(0);
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.set(0);
    }

    public void gethard() {
        shoulderMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
    }

    public ArmPosition getArmPosition() {
        return position;
    }

    public void reportToNetworkTables() {
        SmartDashboard.putNumber("Arm/Shoulder Encoder", shoulderEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Elbow Encoder", elbowEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Wrist Encoder", wristEncoder.getPosition());
    }

    public enum ArmPosition {
        REST(0, 0, 0),
        SHOULDER(20, 0, 0),
        ELBOW(0, -80, 0),
        WRIST(0, 0, 117),
        AMPSHOOTING(0, -100, 120),
        SPEAKERSHOOTING(0, 0, 110),
        TRAPSHOOTING(0, 0, 0),
        INTAKEDEMO(0, 0, 117);

        public final double shoulderAngle;
        public final double elbowAngle;
        public final double wristAngle;

        private ArmPosition(double shoulderAngle, double elbowAngle, double wristAngle) {
            this.shoulderAngle = shoulderAngle;
            this.elbowAngle = elbowAngle;
            this.wristAngle = wristAngle;
        }
    }

}
