// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.controllers;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.team_8840_lib.IO.devices.IOCANCoder;
import frc.team_8840_lib.controllers.SwerveModule.Position;
import frc.team_8840_lib.controllers.specifics.SparkMaxEncoderWrapper;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.async.Promise;
import frc.team_8840_lib.utils.controllers.swerve.CTREModuleState;
import frc.team_8840_lib.utils.controllers.swerve.ModuleConfig;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.math.units.Unit;
import frc.team_8840_lib.utils.math.units.Unit.Type;

public class SwerveModule {
    private SwerveSettings m_settings;
    private ModuleConfig m_config;
    private Position m_position;
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turnMotor;
    private IOCANCoder m_encoder;
    private SparkMaxEncoderWrapper m_driveEncoderWrapper;
    private SparkMaxEncoderWrapper m_turnEncoderWrapper;
    private SparkPIDController m_drivePIDController;
    private SparkPIDController m_turnPIDController;
    private Rotation2d m_lastDesiredAngle;
    private Unit m_lastDesiredSpeed;
    private SimpleMotorFeedforward m_feedforward;
    private double m_drivePositionConversionFactor;
    private double m_driveVelocityConversionFactor;
    private double m_turnPositionConversionFactor;
    private double m_turnVelocityConversionFactor;
    private boolean m_isInitialized = false;

    public SwerveModule(SwerveSettings settings, ModuleConfig config, Position position) {
        this.m_settings = settings;
        this.m_config = config;
        this.m_position = position;
        this.m_driveMotor = new CANSparkMax(this.m_config.getDriveMotorID(), MotorType.kBrushless);
        this.m_turnMotor = new CANSparkMax(this.m_config.getTurnMotorID(), MotorType.kBrushless);
        this.m_encoder = new IOCANCoder(new Object[] { this.m_config.getEncoderID() });
        this.m_encoder.setReal(Robot.isReal());
        if (Robot.isSimulation()) {
            this.m_encoder.setCache(0.0);
        }

        this.m_driveEncoderWrapper = new SparkMaxEncoderWrapper(this.m_driveMotor,
                "Swerve-" + this.m_position.name() + "-Drive");
        this.m_turnEncoderWrapper = new SparkMaxEncoderWrapper(this.m_turnMotor,
                "Swerve-" + this.m_position.name() + "-Turn");
        this.m_drivePIDController = this.m_driveMotor.getPIDController();
        this.m_turnPIDController = this.m_turnMotor.getPIDController();
        this.m_feedforward = new SimpleMotorFeedforward(settings.driveKS, settings.driveKV, settings.driveKA);
        int startInitialization = (int) System.currentTimeMillis();
        (new Promise((res, rej) -> {
            this.configCANCoder();
            Promise.WaitThen(() -> {
                return this.m_encoder.getAbsolutePosition() != 0.0 || Robot.isSimulation();
            }, res, rej, 10);
        })).then((res, rej) -> {
            Promise configPromise = this.configMotors();
            Promise.WaitThen(() -> {
                return configPromise.resolved();
            }, res, rej, 10);
        }).then((res, rej) -> {
            this.m_lastDesiredSpeed = new Unit(0.0, Type.METERS);
            this.m_lastDesiredAngle = this.getAngle();
            res.run();
        }).finish((res, rej) -> {
            int initializationTime = (int) System.currentTimeMillis() - startInitialization;
            Logger.Log(this.m_position.name() + " Swerve Module",
                    "Initialized in " + initializationTime / 1000 + " seconds!");
            this.m_isInitialized = true;
        });
    }

    public void configCANCoder() {
        this.m_encoder.configFactoryDefault();
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.sensorDirection = this.m_settings.canCoderInverted;
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        this.m_encoder.configAllSettings(encoderConfig);
    }

    public Promise configMotors() {
        this.m_turnEncoderWrapper.setManualConversion(this.m_settings.doManualConversion);
        this.m_driveEncoderWrapper.setManualConversion(this.m_settings.doManualConversion);
        this.m_turnMotor.restoreFactoryDefaults();
        this.m_driveMotor.restoreFactoryDefaults();
        this.m_turnPIDController.setFeedbackDevice(this.m_turnEncoderWrapper.getEncoder());
        this.m_drivePIDController.setFeedbackDevice(this.m_driveEncoderWrapper.getEncoder());
        this.m_turnMotor.setInverted(this.m_settings.turnInverted);
        this.m_driveMotor.setInverted(this.m_settings.driveInverted);
        this.m_turnMotor.setIdleMode(this.m_settings.turnIdleMode);
        this.m_driveMotor.setIdleMode(this.m_settings.driveIdleMode);
        this.m_turnMotor.setSmartCurrentLimit((int) Math.round(this.m_settings.turnCurrentLimit.continuousCurrent));
        this.m_driveMotor.setSmartCurrentLimit((int) Math.round(this.m_settings.driveCurrentLimit.continuousCurrent));
        this.m_driveMotor.setSecondaryCurrentLimit(
                (double) ((int) Math.round(this.m_settings.secondaryDriveCurrentLimit.continuousCurrent)));
        this.m_turnMotor.setSecondaryCurrentLimit(
                (double) ((int) Math.round(this.m_settings.secondaryTurnCurrentLimit.continuousCurrent)));
        this.m_turnMotor.enableVoltageCompensation(this.m_settings.voltageCompensation);
        this.m_driveMotor.enableVoltageCompensation(this.m_settings.voltageCompensation);
        this.m_driveMotor.setOpenLoopRampRate(this.m_settings.driveOpenRampRate);
        this.m_driveMotor.setClosedLoopRampRate(this.m_settings.driveClosedRampRate);
        this.m_drivePositionConversionFactor = this.m_settings.driveGearRatio * Math.PI * this.m_settings.wheelDiameter;
        this.m_driveEncoderWrapper.setPositionConversionFactor(this.m_drivePositionConversionFactor);
        this.m_driveVelocityConversionFactor = this.m_drivePositionConversionFactor / 60.0;
        this.m_driveEncoderWrapper.setVelocityConversionFactor(this.m_driveVelocityConversionFactor);
        this.m_turnPositionConversionFactor = 1.0 / this.m_settings.angleGearRatio * 360.0;
        this.m_turnEncoderWrapper.setPositionConversionFactor(this.m_turnPositionConversionFactor);
        this.m_turnVelocityConversionFactor = this.m_turnPositionConversionFactor / 60.0;
        this.m_turnEncoderWrapper.setVelocityConversionFactor(this.m_turnVelocityConversionFactor);
        this.m_turnPIDController.setP(this.m_settings.turnPID.kP);
        this.m_turnPIDController.setI(this.m_settings.turnPID.kI);
        this.m_turnPIDController.setD(this.m_settings.turnPID.kD);
        this.m_turnPIDController.setFF(this.m_settings.turnPID.kF);
        this.m_turnPIDController.setIZone(this.m_settings.turnPID.kIZone);
        this.m_drivePIDController.setP(this.m_settings.drivePID.kP);
        this.m_drivePIDController.setI(this.m_settings.drivePID.kI);
        this.m_drivePIDController.setD(this.m_settings.drivePID.kD);
        this.m_drivePIDController.setFF(this.m_settings.drivePID.kF);
        this.m_drivePIDController.setIZone(this.m_settings.drivePID.kIZone);
        this.m_turnPIDController.setPositionPIDWrappingEnabled(true);
        this.m_turnPIDController
                .setPositionPIDWrappingMinInput(this.m_turnEncoderWrapper.calculatePosition(-180.0, true));
        this.m_turnPIDController
                .setPositionPIDWrappingMaxInput(this.m_turnEncoderWrapper.calculatePosition(180.0, true));
        this.m_driveMotor.burnFlash();
        this.m_turnMotor.burnFlash();
        return (new Promise((res, rej) -> {
            Promise waitFor = this.resetToAbsolute();
            Promise.WaitThen(() -> {
                return waitFor.resolved();
            }, res, rej, 10);
        })).finish((res, rej) -> {
            Logger.Log(this.toString(), "Configured motors!");
            this.m_driveEncoderWrapper.getEncoder().setPosition(0.0);
            this.setNeoCANStatusFrames(this.m_driveMotor, 10, 20, 500, 500, 500);
            this.setNeoCANStatusFrames(this.m_turnMotor, 10, 500, 20, 500, 500);
            if (RobotBase.isSimulation()) {
                REVPhysicsSim.getInstance().addSparkMax(this.m_driveMotor, DCMotor.getNEO(1));
                REVPhysicsSim.getInstance().addSparkMax(this.m_turnMotor, DCMotor.getNEO(1));
            }

            res.run();
        });
    }

    public Promise resetToAbsolute() {
        // this.m_turnEncoderWrapper.getEncoder().setPosition(0.0);
        double newPosition = this.getAbsoluteAngle().getDegrees() - this.m_config.getTurnOffset();
        Logger.Log(this.toString(), "Resetting to newPosition=" + newPosition + " from angle="
                + this.getAbsoluteAngle().getDegrees() + " and offset=" + this.m_config.getTurnOffset());
        return (new Promise((res, rej) -> {
            if (Robot.isSimulation()) {
                this.m_turnEncoderWrapper.setManualOffset(true);
                this.m_turnEncoderWrapper.setPosition(0.0);
                res.run();
            } else if (this.m_config.manualOffset) {
                rej.onError(new Exception("Skipped to wrapper setup because manual offset is enabled!"));
            } else {
                int start = (int) System.currentTimeMillis();
                this.m_turnEncoderWrapper.setPosition(newPosition);
                Promise.WaitThen(() -> {
                    if (Robot.isSimulation()) {
                        return true;
                    } else if ((long) (start + 1000) < System.currentTimeMillis()) {
                        Logger.Log(this.toString(), "Failed to reset to absolute!");
                        throw new RuntimeException(this.toString() + ": Failed to reset to absolute!");
                    } else {
                        return Math.abs(this.m_turnEncoderWrapper.getEncoder().getPosition() - newPosition) < 0.1
                                || Robot.isSimulation();
                    }
                }, res, rej, 10);
            }
        })).then((res, rej) -> {
            Logger.Log(this.toString(), "Successfully reset to absolute through REV API! "
                    + (Robot.isSimulation() ? "(Process fast due to simulation.)" : ""));
            res.run();
        }).catch_err((e) -> {
            // this.m_turnEncoderWrapper.doSubtractionOfStart(true);
            // double newPosition = this.getAbsoluteAngle().getDegrees() -
            // this.m_config.getTurnOffset();
            // this.m_turnEncoderWrapper.setManualOffset(true);
            // this.m_turnEncoderWrapper.setPosition(newPosition);
            Logger.Log(this.toString(),
                    "Was unable to reset to absolute through REV API, using fallback method! (Confirmation: "
                            + this.m_turnEncoderWrapper.getPosition() + " should be equal to 0!)");
        });
    }

    public void setNeoCANStatusFrames(CANSparkMax m_motor, int CANStatus0, int CANStatus1, int CANStatus2,
            int CANStatus3, int CANStatus4) {
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
        m_motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    }

    public void setSpeed(Unit speed, boolean openLoop) {
        this.setSpeed(speed, openLoop, false);
    }

    public void setSpeed(Unit speed, boolean openLoop, boolean ignoreSpeedFlags) {
        double speedDifference = Math.abs(speed.get(Type.METERS) - this.m_lastDesiredSpeed.get(Type.METERS));
        if (!(speedDifference < 0.01) || !(Math.abs(speed.get(Type.METERS)) > 0.1) || ignoreSpeedFlags) {
            if (openLoop) {
                double speedPercentage = speed.get(Type.METERS) / this.m_settings.maxSpeed.get(Type.METERS);
                if (Math.abs(speedPercentage) > 1.0) {
                    speedPercentage = Math.signum(speedPercentage);
                }

                this.m_driveMotor.set(speed.get(Type.METERS) / this.m_settings.maxSpeed.get(Type.METERS));
            } else {
                this.m_drivePIDController.setReference(speed.get(Type.METERS), ControlType.kVelocity, 0,
                        this.m_feedforward.calculate(speed.get(Type.METERS)));
            }

        }
    }

    public void setAngle(Rotation2d angle, boolean ignoreAngleLimit) {
        double difference = Math.abs(angle.getDegrees() - this.m_lastDesiredAngle.getDegrees());
        if (!(difference > 179.0) && !(difference < 0.2) || ignoreAngleLimit) {
            this.m_turnPIDController.setReference(this.m_turnEncoderWrapper.calculatePosition(angle.getDegrees()),
                    ControlType.kPosition, 0, this.m_settings.turnPID.kF);
        }
    }

    public void setDesiredState(SwerveModuleState state, boolean openLoop) {
        this.setDesiredState(state, openLoop, true);
    }

    public void setDesiredState(SwerveModuleState state, boolean openLoop, boolean runOptimization) {
        SwerveModuleState optimizedState = runOptimization ? CTREModuleState.optimize(state, this.m_lastDesiredAngle)
                : state;
        Logger.Log(m_position + " setDesiredState to: " + optimizedState.angle + " was: " + m_lastDesiredAngle);
        this.setSpeed(new Unit(optimizedState.speedMetersPerSecond, Type.METERS), openLoop, runOptimization);
        this.setAngle(optimizedState.angle, runOptimization);
        this.m_lastDesiredAngle = optimizedState.angle;
    }

    public void stop() {
        this.setSpeed(new Unit(0.0, Type.METERS), true, true);
    }

    public Rotation2d getAbsoluteAngle() {
        return Rotation2d.fromDegrees(this.m_encoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.m_driveEncoderWrapper.getVelocity(),
                Rotation2d.fromDegrees(this.m_turnEncoderWrapper.getPosition()));
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(this.m_turnEncoderWrapper.getPosition());
    }

    public Rotation2d getDesiredAngle() {
        return this.m_lastDesiredAngle;
    }

    public Unit getSpeed() {
        return new Unit(this.m_driveEncoderWrapper.getVelocity(), Type.METERS);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.m_driveEncoderWrapper.getPosition(), this.getAngle());
    }

    public boolean initalized() {
        return this.m_isInitialized;
    }

    public String toString() {
        return "SwerveModule:" + this.m_position.name();
    }
}
