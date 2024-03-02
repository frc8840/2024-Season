package frc.robot;

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
    private Climber climber;
    private NewSwerve swerve;
    private PickUpNote intake;
    private ArmShooter outtake;

    public static RobotContainer getInstance() {
        return instance;
    }

    public RobotContainer() {
        instance = this;
        arm = new Arm();
        climber = new Climber();
        // intake = new PickUpNote();
        // outtake = new ArmShooter();
        OperatorControl operatorControl = new OperatorControl(arm, climber, intake, outtake);
        climber.setDefaultCommand(operatorControl);
        // intake.setDefaultCommand(operatorControl);
        // outtake.setDefaultCommand(operatorControl);

        swerve = new NewSwerve();
        Logger.Log("finished making NewSwerve with " + swerve.getPositions());
        DriverControl driverControl = new DriverControl(swerve);
        swerve.setDefaultCommand(driverControl);
    }

}