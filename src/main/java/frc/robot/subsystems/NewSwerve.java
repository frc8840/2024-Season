package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.team_8840_lib.info.console.Logger;

public class NewSwerve extends SubsystemBase {

    private final Pigeon2 gyro;

    private SwerveDriveOdometry odometer;
    private NewSwerveModule[] mSwerveMods;

    private Field2d field;

    public NewSwerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        zeroGyro();

        SwerveModulePosition[] startPositions = new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
        };
        odometer = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, new Rotation2d(0), startPositions);

        // order is always FRONT LEFT, FRONT RIGHT, BACK LEFT, BACK RIGHT
        mSwerveMods = new NewSwerveModule[] {
                new NewSwerveModule(0, Constants.Swerve.FLconstants),
                new NewSwerveModule(1, Constants.Swerve.FRconstants),
                new NewSwerveModule(2, Constants.Swerve.BLconstants),
                new NewSwerveModule(3, Constants.Swerve.BRconstants)
        };

        field = new Field2d();
        SmartDashboard.putData("Field", field);
        // Logger.Log("mSwerveMods.length=" + mSwerveMods.length);

        // starting to use pathplanner
        Logger.Log("about to configureHolonomic");
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(0.01, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0.01, 0.0, 0.0), // Rotation PID constants
                        0.5, // Max module speed, in m/s
                        0.6, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
        Logger.Log("completed configureHolonomic");
    }

    // translation and rotation are the desired behavior of the robot at this moment
    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative) {

        // first, we compute our desired chassis speeds
        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        drive(chassisSpeeds);
    }

    // used by tele
    public void drive(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        // do we need the below?
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        setModuleStates(swerveModuleStates);
    }

    // used by auto
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (NewSwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getYaw(), getPositions(), pose);
    }

    public NewSwerveModule[] getModules() {
        return mSwerveMods;
    }

    public SwerveModulePosition[] getPositions() {
        if (mSwerveMods == null || mSwerveMods.length != 4) {
            // Logger.Log("mSwerveMods = null");
            return null;
        }
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < mSwerveMods.length; i++) {
            positions[i] = mSwerveMods[i].getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        // Logger.Log("zeroGyro called");
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro)
                ? Rotation2d.fromDegrees(360 - gyro.getAngle())
                : Rotation2d.fromDegrees(gyro.getAngle());
    }

    public enum DrivePosition {
        // add in PID commands later
        MOVELEFT(),
        MOVERIGHT(),
        MOVEFORWARD(),
        MOVEBACK();
    }

    @Override
    public void periodic() {
        odometer.update(getYaw(), getPositions());
        field.setRobotPose(getPose());

        // for (NewSwerveModule mod : mSwerveMods) {
        // SmartDashboard.putNumber(
        // "Mod " + mod.moduleNumber + " Cancoder",
        // mod.getCanCoderAngle().getDegrees());
        // }
        // tell dashboard where the robot thinks it is
        SmartDashboard.putNumber(
                "Robot heading:", gyro.getAngle());
        SmartDashboard.putString(
                "Robot location:", getPose().getTranslation().toString());
    }

    public void stopModules() {
        for (NewSwerveModule mod : mSwerveMods) {
            mod.stop();
        }
    }

    // constructs current estimate of chassis speeds from module encoders
    public ChassisSpeeds getChassisSpeeds() {
        var frontLeftState = mSwerveMods[0].getState();
        var frontRightState = mSwerveMods[1].getState();
        var backLeftState = mSwerveMods[2].getState();
        var backRightState = mSwerveMods[3].getState();

        // Convert to chassis speeds
        ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(
                frontLeftState, frontRightState, backLeftState, backRightState);

        return chassisSpeeds;
    }
}