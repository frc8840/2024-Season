package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

        public static final class Climber {
                public static final double climberKP = 0.1;
                public static final double climberKI = 0.0;
                public static final double climberKD = 0.02;
                public static final double climberKFF = 0.0;

        }

        public static final class Swerve {

                public static final int pigeonID = 42;
                public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

                /* Drivetrain Constants */
                // our robot this year is longer than wide
                public static final double trackWidth = Units.inchesToMeters(22.75); // width 22.75
                public static final double wheelBase = Units.inchesToMeters(24.75); // length 24.75
                public static final double wheelDiameter = Units.inchesToMeters(4.0);
                public static final double wheelCircumference = wheelDiameter * Math.PI;

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                public static final double driveGearRatio = (6.75 / 1.0); // SDS website for MK4i says 6.75:1
                public static final double angleGearRatio = (21.4 / 1.0); // SDS website for MK4i says 150/7:1 (was
                                                                          // 12.8:1)

                // assuming coordinates from WPILIB from here:
                // https://www.chiefdelphi.com/t/swerve-x-and-y-flipped-in-odometry/451670/2
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // front right
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front left
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // back right
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0) // back left
                );

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 20;
                public static final int driveContinuousCurrentLimit = 40;

                /* Angle Motor PID Values */
                public static final double angleKP = 0.01;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.0;
                public static final double angleKFF = 0.00;

                /* Drive Motor PID Values */
                public static final double driveKP = 0.1;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = 0.0;

                /* Drive Motor Characterization Values */
                public static final double driveKS = 0.667;
                public static final double driveKV = 2.44;
                public static final double driveKA = 0.27;

                /* Drive Motor Conversion Factors */
                public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
                public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
                public static final double angleConversionFactor = 360.0 / angleGearRatio;

                /* Swerve Profiling Values */
                public static final double maxSpeed = 1.8; // meters per second
                public static final double maxAngularVelocity = 5.0;

                /* Neutral Modes */
                public static final CANSparkMax.IdleMode angleNeutralMode = CANSparkMax.IdleMode.kBrake;
                public static final CANSparkMax.IdleMode driveNeutralMode = CANSparkMax.IdleMode.kBrake;

                /* Motor Inverts */
                public static final boolean driveInvert = false;
                public static final boolean angleInvert = false;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final SwerveModuleConstants FLconstants = new SwerveModuleConstants(12, 11,
                                21, Rotation2d.fromDegrees(128.0));

                /* Front Right Module - Module 1 */
                public static final SwerveModuleConstants FRconstants = new SwerveModuleConstants(14, 13, 22,
                                Rotation2d.fromDegrees(339.0));

                /* Back Left Module - Module 2 */
                public static final SwerveModuleConstants BLconstants = new SwerveModuleConstants(18, 17, 24,
                                Rotation2d.fromDegrees(200.0));

                /* Back Right Module - Module 3 */
                public static final SwerveModuleConstants BRconstants = new SwerveModuleConstants(16, 15, 23,
                                Rotation2d.fromDegrees(50.0));
        }

        public static final class AutoConstants {
                public static final double kMaxSpeedMetersPerSecond = 0.5;
                public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
                public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

                public static final double kPXController = 1;
                public static final double kPYController = 1;
                public static final double kPThetaController = 1;

                // // Constraint for the motion profilied robot angle controller
                public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                                kMaxAngularSpeedRadiansPerSecond,
                                kMaxAngularAccelerationRadiansPerSecondSquared);
        }
}