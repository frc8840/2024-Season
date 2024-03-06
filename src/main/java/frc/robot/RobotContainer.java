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

        // construct the subsystems
        swerve = new NewSwerve();
        arm = new Arm();
        climber = new Climber();
        intake = new PickUpNote();
        outtake = new ArmShooter();

        Logger.Log("finished constructing subsystems, going to sleep");
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            Logger.Log("sleep interrupted");
        }
        Logger.Log("finished sleeping");

        // now make the controls
        DriverControl driverControl = new DriverControl(swerve, arm);
        swerve.setDefaultCommand(driverControl);

        OperatorControl operatorControl = new OperatorControl(arm, climber, intake, outtake);
        climber.setDefaultCommand(operatorControl);
    }

}