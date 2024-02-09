package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.team_8840_lib.controllers.specifics.SparkMaxEncoderWrapper;
import frc.team_8840_lib.listeners.Robot;

public class Arm extends SubsystemBase {
    private CANSparkMax baseMotor;
    private CANSparkMax elbowMotor;
    private SparkMaxEncoderWrapper baseEncoder;
    private SparkMaxEncoderWrapper elbowEncoder;
    private SparkPIDController basePID;
    private SparkPIDController elbowPID;
    private ArmPosition position = ArmPosition.REST;

    public Arm() {
        baseMotor = new CANSparkMax(Settings.BASE_MOTOR_ID, MotorType.kBrushless);
        elbowMotor = new CANSparkMax(Settings.ELBOW_MOTOR_ID, MotorType.kBrushless);

        baseEncoder = new SparkMaxEncoderWrapper(baseMotor);
        elbowEncoder = new SparkMaxEncoderWrapper(elbowMotor);

        baseEncoder.setManualOffset(true);
        baseEncoder.setPosition(0);
        baseEncoder.setManualConversion(Robot.isSimulation());

        elbowEncoder.setManualOffset(true);
        elbowEncoder.setPosition(0);
        elbowEncoder.setManualConversion(Robot.isSimulation());

        baseMotor.restoreFactoryDefaults();
        elbowMotor.restoreFactoryDefaults();

        baseMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        elbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        baseMotor.setSmartCurrentLimit(25);
        baseMotor.setSecondaryCurrentLimit(30);

        elbowMotor.setSmartCurrentLimit(25);
        elbowMotor.setSecondaryCurrentLimit(30);

        baseMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);
        elbowMotor.setClosedLoopRampRate(Settings.CLOSED_LOOP_RAMP_RATE);

        baseMotor.enableVoltageCompensation(12);
        elbowMotor.enableVoltageCompensation(12);

        double positionConversionFactor = (1 / Settings.GEAR_RATIO) * 360;
        baseEncoder.setPositionConversionFactor(positionConversionFactor);
        elbowEncoder.setPositionConversionFactor(positionConversionFactor);

        double velocityConversionFactor = positionConversionFactor / 60;
        baseEncoder.setVelocityConversionFactor(velocityConversionFactor);
        elbowEncoder.setVelocityConversionFactor(velocityConversionFactor);

        basePID = baseMotor.getPIDController();
        elbowPID = elbowMotor.getPIDController();

        basePID.setP(Settings.BASE_PID.kP);
        basePID.setI(Settings.BASE_PID.kI);
        basePID.setD(Settings.BASE_PID.kD);
        basePID.setIZone(Settings.BASE_PID.kIZone);
        basePID.setFF(Settings.BASE_PID.kF);

        elbowPID.setP(Settings.ELBOW_PID.kP);
        elbowPID.setI(Settings.ELBOW_PID.kI);
        elbowPID.setD(Settings.ELBOW_PID.kD);
        elbowPID.setIZone(Settings.ELBOW_PID.kIZone);
        elbowPID.setFF(Settings.ELBOW_PID.kF);

        basePID.setOutputRange(-Settings.MAX_BASE_SPEED, Settings.MAX_BASE_SPEED);
        elbowPID.setOutputRange(-Settings.MAX_ELBOW_SPEED, Settings.MAX_ELBOW_SPEED);

        basePID.setFeedbackDevice(baseEncoder.getEncoder());
        elbowPID.setFeedbackDevice(elbowEncoder.getEncoder());

        baseMotor.burnFlash();
        elbowMotor.burnFlash();
    }

    public void setArmPosition(ArmPosition position) {
        this.position = position;

        basePID.setReference(
                baseEncoder.calculatePosition(position.baseAngle),
                ControlType.kPosition,
                0);

        elbowPID.setReference(
                elbowEncoder.calculatePosition(position.elbowAngle),
                ControlType.kPosition,
                0);
    }

    public void setBaseSpeed(double speed) {
        baseMotor.set(speed);
    }

    public void setElbowSpeed(double speed) {
        elbowMotor.set(speed);
    }

    public ArmPosition getArmPosition() {
        return position;
    }

    public void reportToNetworkTables() {
        SmartDashboard.putNumber("Arm/Base Encoder", baseEncoder.getPosition());
        SmartDashboard.putNumber("Arm/Elbow Encoder", elbowEncoder.getPosition());
    }

    public enum ArmPosition {
        REST(0, 0),
        GROUND_INTAKE(0, 0),
        DOUBLE_SUBSTATION_INTAKE(0, 0),
        HYBRID(0, 0),
        MID_CONE(0, 0),
        HIGH_CONE(0, 0);

        public final double baseAngle;
        public final double elbowAngle;

        private ArmPosition(double baseAngle, double elbowAngle) {
            this.baseAngle = baseAngle;
            this.elbowAngle = elbowAngle;
        }
    }

}
