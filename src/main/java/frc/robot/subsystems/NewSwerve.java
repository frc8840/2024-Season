package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team_8840_lib.info.console.Logger;

public class NewSwerve extends SubsystemBase {
    private final Pigeon2 gyro;

    private SwerveDriveOdometry swerveOdometry;
    private NewSwerveModule[] mSwerveMods;
    private SwerveModulePosition[] startPositions;

    private Field2d field;

    public NewSwerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        startPositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
        };
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), startPositions);

        mSwerveMods = new NewSwerveModule[] {
                new NewSwerveModule(0, Constants.Swerve.Mod0.constants),
                new NewSwerveModule(1, Constants.Swerve.Mod1.constants),
                new NewSwerveModule(2, Constants.Swerve.Mod2.constants),
                new NewSwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        field = new Field2d();
        SmartDashboard.putData("Field", field);
        Logger.Log("mSwerveMods.length=" + mSwerveMods.length);
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(), translation.getY(), rotation, getYaw())
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (NewSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (NewSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), startPositions, pose);
    }

    public NewSwerveModule[] getModules() {
        return mSwerveMods;
    }

    public SwerveModulePosition[] getStates() {
        if (mSwerveMods == null || mSwerveMods.length != 4) {
            Logger.Log("mSwerveMods = null");
            return null;
        }
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            states[i] = mSwerveMods[i].getState();
        }
        return states;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getStates());
        field.setRobotPose(getPose());

        for (NewSwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Integrated",
                    mod.getState().angle.getDegrees());
            SmartDashboard.putNumber(
                    "Mod " + mod.moduleNumber + " Velocity", mod.getState().distanceMeters);
        }
    }
}