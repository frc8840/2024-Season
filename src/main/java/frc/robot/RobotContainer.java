package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.PickUpNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.ArmShooter;
import frc.team_8840_lib.info.console.Logger;

public class RobotContainer {
    private static RobotContainer instance;
    private Arm arm;
    private Climber climber;
    private NewSwerve swerve;
    private PickUpNote intake;
    private ArmShooter shooter;

    // controllers
    DriverControl driverControl;
    OperatorControl operatorControl;

    // for autonomous
    TrajectoryConfig trajectoryConfig;

    public static RobotContainer getInstance() {
        return instance;
    }

    public RobotContainer() {
        instance = this;

        // construct the subsystems
        swerve = new NewSwerve();
        arm = new Arm();
        climber = new Climber();
        intake = new PickUpNote();
        shooter = new ArmShooter();

        Logger.Log("finished constructing subsystems, going to sleep");
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Logger.Log("sleep interrupted");
        }
        Logger.Log("finished sleeping");

        // now make the controllers
        driverControl = new DriverControl(swerve, arm);
        swerve.setDefaultCommand(driverControl);

        operatorControl = new OperatorControl(arm, climber, intake, shooter);
        climber.setDefaultCommand(operatorControl);

        // now make the trajectory config for auto
        trajectoryConfig = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

    }

    public Command getA2BlueCommand() {
        trajectoryConfig.setReversed(true);
        Trajectory t1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(Math.PI / 3)),
                List.of(
                // new Translation2d(-0.254, 1.524)
                ),

                new Pose2d(-0.254, 1.524, new Rotation2d(0)),

                trajectoryConfig);
        trajectoryConfig.setReversed(false);
        Trajectory t2 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.254, 1.524, new Rotation2d(0)),
                List.of(
                // new Translation2d ()
                ),
                new Pose2d(0, 0, new Rotation2d(Math.PI / 3)),
                trajectoryConfig);
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
                new InstantCommand(() -> shooter.shoot()),
                new WaitCommand(2),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    intake.stop();
                    shooter.stop();
                }),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
                new WaitCommand(0.5),
                getAutonomousCommand(t1),
                new InstantCommand(() -> swerve.stopModules()),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.WRIST)),
                new WaitCommand(1),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(0.5),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
                new WaitCommand(0.5),
                getAutonomousCommand(t2),
                new InstantCommand(() -> swerve.stopModules()),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
                new WaitCommand(0.5),
                new InstantCommand(() -> shooter.shoot()),
                new WaitCommand(2),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    intake.stop();
                    shooter.stop();
                }));

    }

    public Command shootAndDriveCommand(SimpleDirection direction) {
        // get the pose for the direction
        Pose2d pose = new Pose2d(-2, 0, new Rotation2d(0)); // straight
        if (direction == SimpleDirection.diagonalLeft) {
            pose = new Pose2d(-1.4, 1.4, new Rotation2d(0));
        } else if (direction == SimpleDirection.diagonalRight) {
            pose = new Pose2d(-1.4, -1.4, new Rotation2d(0));
        }

        intake.inComplexAction = true;
        // before we make our trajectory, let it know that we should go in reverse
        trajectoryConfig.setReversed(true);
        Trajectory rollForward2Meters = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(),
                pose, // the trajectory generator seems to think we have to go bakcwards
                trajectoryConfig);
        Trajectory rollBackward2Meters = TrajectoryGenerator.generateTrajectory(
                pose,
                List.of(),
                new Pose2d(0, 0, new Rotation2d(0)), // the trajectory generator seems to think we have to go bakcwards
                trajectoryConfig);
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
                new InstantCommand(() -> shooter.shoot()),
                new WaitCommand(2),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    intake.stop();
                    shooter.stop();
                }),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
                getAutonomousCommand(rollForward2Meters),
                new WaitCommand(2),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.WRIST)),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(1),
                getAutonomousCommand(rollBackward2Meters),
                new WaitCommand(2),
                new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
                new InstantCommand(() -> shooter.shoot()),
                new WaitCommand(2),
                new InstantCommand(() -> intake.intake()),
                new WaitCommand(1),
                new InstantCommand(() -> {
                    intake.stop();
                    shooter.stop();
                }),
                new InstantCommand(() -> swerve.stopModules()));

    }

    // following the pattern set in this video:
    // https://www.chiefdelphi.com/t/0-to-autonomous-6-swerve-drive-auto/401117
    public SwerveControllerCommand getAutonomousCommand(Trajectory trajectory) {
        // create the PID controllers for feedback
        PIDController xController = new PIDController(0.2, 0, 0);
        PIDController yController = new PIDController(0.2, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0, 0,
                Constants.AutoConstants.kThetaControllerConstraints);
        ;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(
                trajectory,
                swerve::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                swerve::setModuleStates,
                swerve);
    }

    public enum SimpleDirection {
        straight,
        diagonalRight,
        diagonalLeft,
    }

}
