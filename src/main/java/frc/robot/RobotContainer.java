package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.Climber;
import frc.team_8840_lib.info.console.Logger;

public class RobotContainer {
    private static RobotContainer instance;
    private Climber climber;
    private NewSwerve swerve;

    public static RobotContainer getInstance() {
        return instance;
    }

    public RobotContainer() {
        instance = this;
        climber = new Climber();
        OperatorControl operatorControl = new OperatorControl(climber);
        climber.setDefaultCommand(operatorControl);

        swerve = new NewSwerve();
        Logger.Log("finished making NewSwerve with " + swerve.getPositions());
        DriverControl driverControl = new DriverControl(swerve);
        swerve.setDefaultCommand(driverControl);
    }

}