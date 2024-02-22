// Source code is decompiled from a .class file using FernFlower decompiler.
package frc.robot.controllers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team_8840_lib.controllers.SwerveModule.Position;
import frc.team_8840_lib.info.console.AutoLog;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.replay.Replayable;
import frc.team_8840_lib.utils.IO.IOMethod;
import frc.team_8840_lib.utils.IO.IOMethodType;
import frc.team_8840_lib.utils.IO.IOValue;
import frc.team_8840_lib.utils.async.Promise;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.ModuleConfig;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.math.units.Unit;
import frc.team_8840_lib.utils.math.units.Unit.Type;

public class SwerveDrive extends Replayable {
    private SwerveDriveOdometry m_odometry;
    private SwerveSettings m_settings;
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_backLeft;
    private SwerveModule m_backRight;
    private Pigeon m_pigeon;
    private boolean m_isInitialized;
    private boolean m_replayOpenLoop = false;
    private Pigeon m_copy_pigeon;

    public SwerveDrive(ModuleConfig frontLeft, ModuleConfig frontRight, ModuleConfig backLeft, ModuleConfig backRight,
            Pigeon pigeon, SwerveSettings settings) {
        this.m_settings = settings;
        this.m_frontLeft = new SwerveModule(this.m_settings, frontLeft, Position.FRONT_LEFT);
        this.m_frontRight = new SwerveModule(this.m_settings, frontRight, Position.FRONT_RIGHT);
        this.m_backLeft = new SwerveModule(this.m_settings, backLeft, Position.BACK_LEFT);
        this.m_backRight = new SwerveModule(this.m_settings, backRight, Position.BACK_RIGHT);
        this.m_pigeon = pigeon;
        int startTime = (int) System.currentTimeMillis();
        (new Promise((res, rej) -> {
            Promise.WaitThen(() -> {
                return this.m_frontLeft.initalized() && this.m_frontRight.initalized() && this.m_backLeft.initalized()
                        && this.m_backRight.initalized();
            }, res, rej, 10);
        })).then((res, rej) -> {
            this.m_odometry = new SwerveDriveOdometry(settings.getKinematics(), this.getAngle(),
                    this.getSwervePositions());
            Logger.Log(this.getBaseName(), "Enabled odometry!");
        }).then((res, rej) -> {
            int endTime = (int) System.currentTimeMillis();
            Logger.Log(this.getBaseName(), "Initialized in " + (endTime - startTime) + "ms");
            this.m_isInitialized = true;
            res.run();
        }).catch_err((err) -> {
            err.printStackTrace();
            Logger.Log(this.getBaseName(), "Failed to initialize!");
        });
    }

    public void drive(Translation2d translation, Rotation2d rotationSpeed, boolean fieldRelative, boolean openLoop) {
        Logger.Log("drive starting");
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                        rotationSpeed.getRadians(), this.getAngle())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotationSpeed.getRadians());
        SwerveModuleState[] states = this.m_settings.getKinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, this.m_settings.maxSpeed.get(Type.METERS));
        this.setModuleStates(states[0], states[1], states[2], states[3], openLoop, true);
    }

    public void stop() {
        this.m_frontLeft.stop();
        this.m_frontRight.stop();
        this.m_backLeft.stop();
        this.m_backRight.stop();
    }

    public void spin(Rotation2d rotationPerSecond, boolean openLoop) {
        Logger.Log("spin starting");
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, rotationPerSecond.getRadians());
        SwerveModuleState[] states = this.m_settings.getKinematics().toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, this.m_settings.maxSpeed.get(Type.METERS));
        this.setModuleStates(states[0], states[1], states[2], states[3], openLoop, true);
    }

    public void setModuleStates(SwerveModuleState frontRight, SwerveModuleState backRight, SwerveModuleState frontLeft,
            SwerveModuleState backLeft, boolean openLoop, boolean runOptimization) {
        this.m_frontRight.setDesiredState(frontRight, openLoop, runOptimization);
        this.m_backRight.setDesiredState(backRight, openLoop, runOptimization);
        this.m_frontLeft.setDesiredState(frontLeft, openLoop, runOptimization);
        this.m_backLeft.setDesiredState(backLeft, openLoop, runOptimization);
    }

    public SwerveModulePosition[] getSwervePositions() {
        return new SwerveModulePosition[] { this.m_frontRight.getPosition(), this.m_backRight.getPosition(),
                this.m_frontLeft.getPosition(), this.m_backLeft.getPosition() };
    }

    public void updateOdometry() {
        this.m_odometry.update(this.getAngle(), this.getSwervePositions());
    }

    public void resetOdometry(Pose2d pose) {
        this.m_odometry.resetPosition(this.getAngle(), this.getSwervePositions(), pose);
    }

    public Pose2d getPose() {
        return this.m_odometry.getPoseMeters();
    }

    public SwerveSettings getSettings() {
        return this.m_settings;
    }

    public SwerveModule[] getModules() {
        return new SwerveModule[] { this.m_frontRight, this.m_backRight, this.m_frontLeft, this.m_backLeft };
    }

    public Rotation2d getAngle() {
        double yaw = this.m_pigeon.getYawPitchRoll()[0];
        return (this.m_settings.invertGyro ? Rotation2d.fromDegrees(360.0 - yaw) : Rotation2d.fromDegrees(yaw))
                .plus(this.m_settings.gyroscopeStartingAngle);
    }

    public boolean isReady() {
        return this.m_isInitialized;
    }

    public String getBaseName() {
        return "SwerveDrive";
    }

    @AutoLog(name = "Gyroscope", replaylink = "replayGyroscope")
    public double[] getPointing() {
        return this.m_pigeon.getYawPitchRoll();
    }

    @IOMethod(name = "replayGyroscope", value_type = IOValue.DOUBLE_ARRAY, method_type = IOMethodType.WRITE, toNT = false)
    public void replayGyroscope(double[] pointing) {
        if (this.replaying()) {
            this.m_pigeon.setDummyAngle(pointing[0]);
        }
    }

    @AutoLog(name = "openLoop", replaylink = "replayOpenLoop")
    public boolean isOpenLoop() {
        return this.m_replayOpenLoop;
    }

    @IOMethod(name = "replayOpenLoop", value_type = IOValue.BOOLEAN, method_type = IOMethodType.WRITE, toNT = false)
    public void replayOpenLoop(boolean openLoop) {
        if (this.replaying()) {
            this.m_replayOpenLoop = openLoop;
        }
    }

    @AutoLog(name = "Angles", replaylink = "replayAngles")
    public double[] getAngles() {
        return new double[] { this.m_frontRight.getAngle().getDegrees(), this.m_backRight.getAngle().getDegrees(),
                this.m_frontLeft.getAngle().getDegrees(), this.m_backLeft.getAngle().getDegrees() };
    }

    @IOMethod(name = "replayAngles", value_type = IOValue.DOUBLE_ARRAY, method_type = IOMethodType.WRITE, toNT = false)
    public void replayAngles(double[] angles) {
        if (this.replaying()) {
            Rotation2d[] rotAngles = new Rotation2d[] { Rotation2d.fromDegrees(angles[0]),
                    Rotation2d.fromDegrees(angles[1]), Rotation2d.fromDegrees(angles[2]),
                    Rotation2d.fromDegrees(angles[3]) };
            this.m_frontRight.setAngle(rotAngles[0], this.m_isInitialized);
            this.m_backRight.setAngle(rotAngles[1], this.m_isInitialized);
            this.m_frontLeft.setAngle(rotAngles[2], this.m_isInitialized);
            this.m_backLeft.setAngle(rotAngles[3], this.m_isInitialized);
        }
    }

    @AutoLog(name = "Speeds", replaylink = "replaySpeeds")
    public double[] getSpeeds() {
        return new double[] { this.m_frontRight.getSpeed().get(Type.METERS),
                this.m_backRight.getSpeed().get(Type.METERS), this.m_frontLeft.getSpeed().get(Type.METERS),
                this.m_backLeft.getSpeed().get(Type.METERS) };
    }

    @IOMethod(name = "replaySpeeds", value_type = IOValue.DOUBLE_ARRAY, method_type = IOMethodType.WRITE, toNT = false)
    public void replaySpeeds(double[] speeds) {
        if (this.replaying()) {
            this.m_frontRight.setSpeed(new Unit(speeds[0], Type.METERS), this.m_replayOpenLoop);
            this.m_backRight.setSpeed(new Unit(speeds[1], Type.METERS), this.m_replayOpenLoop);
            this.m_frontLeft.setSpeed(new Unit(speeds[2], Type.METERS), this.m_replayOpenLoop);
            this.m_backLeft.setSpeed(new Unit(speeds[3], Type.METERS), this.m_replayOpenLoop);
        }
    }

    public void replayInit() {
        this.m_copy_pigeon = this.m_pigeon;
        this.m_pigeon = new Pigeon(frc.team_8840_lib.utils.controllers.Pigeon.Type.DUMMY, this.m_pigeon.getID());
    }

    public void exitReplay() {
        this.m_pigeon = this.m_copy_pigeon;
        this.m_copy_pigeon = null;
    }
}
