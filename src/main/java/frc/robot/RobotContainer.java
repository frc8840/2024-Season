package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.PickUpNote;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmShooter;
import frc.team_8840_lib.info.console.Logger;

public class RobotContainer {
    private static RobotContainer instance;
    private Arm arm;
    public Climber climber;
    private NewSwerve swerve;
    public PickUpNote intake;
    public ArmShooter shooter;

    // for the choosing stage of pathplanner auto
    private final SendableChooser<Command> autoChooser;

    // controllers
    DriverControl driverControl;
    OperatorControl operatorControl;

    // for autonomous
    // TrajectoryConfig trajectoryConfig;

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
        // trajectoryConfig = new
        // TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        // Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // .setKinematics(Constants.Swerve.swerveKinematics);

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        // isAbletoShoot =

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    // public Command shootAndDriveCommand(SimpleDirection direction) {
    // // get the pose for the direction
    // Pose2d pose = new Pose2d(-0.8636, 0, new Rotation2d(0)); // straight
    // if (direction == SimpleDirection.diagonalLeft) {
    // pose = new Pose2d(-1.4, 1.4, new Rotation2d(0));
    // } else if (direction == SimpleDirection.diagonalRight) {
    // pose = new Pose2d(-1.4, -1.4, new Rotation2d(0));
    // }

    // intake.inComplexAction = true;
    // // before we make our trajectory, let it know that we should go in reverse
    // trajectoryConfig.setReversed(true);
    // Trajectory rollForward2Meters = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, new Rotation2d(0)),
    // List.of(),
    // pose, // the trajectory generator seems to think we have to go bakcwards
    // trajectoryConfig);
    // return new SequentialCommandGroup(
    // new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new
    // Rotation2d(0)))),
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
    // new InstantCommand(() -> shooter.shoot()),
    // new WaitCommand(2),
    // new InstantCommand(() -> intake.intake()),
    // new WaitCommand(1),
    // new InstantCommand(() -> {
    // intake.stop();
    // shooter.stop();
    // }),
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
    // getAutonomousCommand(rollForward2Meters),
    // new InstantCommand(() -> swerve.stopModules()));

    // }

    // public Command shootAndDriveAndShootAgainCommand(SimpleDirection direction) {
    // // get the pose for the direction
    // Pose2d pose = new Pose2d(-1.1136, 0, new Rotation2d(0)); // straight
    // if (direction == SimpleDirection.diagonalLeft) {
    // pose = new Pose2d(-1.4, 1.4, new Rotation2d(0));
    // } else if (direction == SimpleDirection.diagonalRight) {
    // pose = new Pose2d(-1.4, -1.4, new Rotation2d(0));
    // }

    // intake.inComplexAction = true;
    // // before we make our trajectory, let it know that we should go in reverse
    // trajectoryConfig.setReversed(true);
    // Trajectory rollForwardMeters = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, new Rotation2d(0)),
    // List.of(),
    // pose, // the trajectory generator seems to think we have to go bakcwards
    // trajectoryConfig);
    // trajectoryConfig.setReversed(false);
    // Trajectory rollBackwardMeters = TrajectoryGenerator.generateTrajectory(
    // pose,
    // List.of(),
    // new Pose2d(-0.25, 0, new Rotation2d(0)), // the trajectory generator seems to
    // think we have to go
    // // bakcwards
    // trajectoryConfig);
    // return new SequentialCommandGroup(
    // // reset odometry
    // new InstantCommand(() -> swerve.resetOdometry(new Pose2d(0, 0, new
    // Rotation2d(0)))),
    // // shoot the preloaded note
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
    // new InstantCommand(() -> shooter.shoot()),
    // new WaitCommand(2),
    // new InstantCommand(() -> intake.intake()),
    // new WaitCommand(1),
    // new InstantCommand(() -> {
    // // intake.stop();
    // shooter.stop();
    // }),
    // // put intake down
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.WRIST)),
    // // run the intake
    // new WaitCommand(1),

    // // roll forward
    // getAutonomousCommand(rollForwardMeters),

    // // roll backward
    // getAutonomousCommand(rollBackwardMeters),
    // new WaitCommand(0.5),
    // new InstantCommand(() -> {
    // intake.stop();
    // }),
    // // shoot
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.SPEAKERSHOOTING)),
    // new InstantCommand(() -> shooter.shoot()),
    // new WaitCommand(2),
    // new InstantCommand(() -> intake.intake()),
    // new WaitCommand(1),
    // new InstantCommand(() -> {
    // intake.stop();
    // shooter.stop();
    // }),
    // // move arm to rest and stop
    // new InstantCommand(() -> arm.setArmPosition(ArmPosition.REST)),
    // new InstantCommand(() -> swerve.stopModules()));

    // }

    // // following the pattern set in this video:
    // // https://www.chiefdelphi.com/t/0-to-autonomous-6-swerve-drive-auto/401117
    // public SwerveControllerCommand getAutonomousCommand(Trajectory trajectory) {
    // // create the PID controllers for feedback
    // PIDController xController = new PIDController(0.2, 0, 0);
    // PIDController yController = new PIDController(0.2, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0, 0,
    // Constants.AutoConstants.kThetaControllerConstraints);
    // ;
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // return new SwerveControllerCommand(
    // trajectory,
    // swerve::getPose,
    // Constants.Swerve.swerveKinematics,
    // xController,
    // yController,
    // thetaController,
    // swerve::setModuleStates,
    // swerve);
    // }

    // public enum SimpleDirection {
    // straight,
    // diagonalRight,
    // diagonalLeft,
    // }

    // it gets the selection from smart dashboard
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
