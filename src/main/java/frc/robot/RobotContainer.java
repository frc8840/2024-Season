package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.PickUpNote;
import frc.robot.subsystems.Arm.ArmPosition;
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

        // now we set up things for auto selection and pathplanner
        // these are commands that the path from pathplanner will use
        NamedCommands.registerCommand("Start Intake", getStartIntakeCommand());
        NamedCommands.registerCommand("Stop Intake", getStopIntakeCommand());
        NamedCommands.registerCommand("Shoot", getShootCommand());

        // Build an auto chooser and put it on the SmartDashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    public Command getStartIntakeCommand() {
        return new InstantCommand(() -> {
            arm.setArmPosition(ArmPosition.INTAKE);
            intake.intake();
        });
    }

    public Command getStopIntakeCommand() {
        return new InstantCommand(() -> {
            intake.stop();
            arm.setArmPosition(ArmPosition.REST);
        });
    }

    public Command getShootCommand() {
        return new InstantCommand(() -> {
            arm.setArmPosition(ArmPosition.SPEAKERSHOOTING);
            shooter.shoot();
        });
    }

    // it gets the selection from smart dashboard
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
