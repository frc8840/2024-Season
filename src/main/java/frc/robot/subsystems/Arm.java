package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

        shoulderMotor.setSmartCurrentLimit(25);
        shoulderMotor.setSecondaryCurrentLimit(30);

        elbowMotor.setSmartCurrentLimit(25);
        elbowMotor.setSecondaryCurrentLimit(30);

        wristMotor.setSmartCurrentLimit(25);
        wristMotor.setSecondaryCurrentLimit(30);

        shoulderMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        elbowMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        wristMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);

        shoulderMotor.enableVoltageCompensation(12);
        elbowMotor.enableVoltageCompensation(12);
        wristMotor.enableVoltageCompensation(12);

        double positionConversionFactor = (1 / Settings.GEAR_RATIO) * 360;
        shoulderEncoder.setPositionConversionFactor(positionConversionFactor);
        elbowEncoder.setPositionConversionFactor(positionConversionFactor);
        wristEncoder.setPositionConversionFactor(positionConversionFactor);

        double velocityConversionFactor = positionConversionFactor / 60;
        shoulderEncoder.setVelocityConversionFactor(velocityConversionFactor);
        elbowEncoder.setVelocityConversionFactor(velocityConversionFactor);
        wristEncoder.setVelocityConversionFactor(velocityConversionFactor);

        shoulderPID = shoulderMotor.getPIDController();
        elbowPID = elbowMotor.getPIDController();
        wristPID = elbowMotor.getPIDController();

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

        shoulderPID.setFeedbackDevice(shoulderEncoder);
        elbowPID.setFeedbackDevice(elbowEncoder);
        wristPID.setFeedbackDevice(wristEncoder);

        shoulderMotor.burnFlash();
        elbowMotor.burnFlash();
        wristMotor.burnFlash();
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Logger.Log("interrupted");
        }
    }

    public void setArmPosition(ArmPosition position) {
        this.position = position;

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

    public void setShoulderSpeed(double speed) {
        shoulderMotor.set(speed);
    }

    public void setElbowSpeed(double speed) {
        elbowMotor.set(speed);
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
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
        TEST(90, 0, 0),
        DOUBLE_SUBSTATION_INTAKE(0, 0, 0),
        HYBRID(0, 0, 0),
        MID_CONE(0, 0, 0),
        HIGH_CONE(0, 0, 0);

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
