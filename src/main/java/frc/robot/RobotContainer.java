package frc.robot;

import frc.robot.Commands.OperatorControl;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Swerve;

public class RobotContainer {
    private static RobotContainer instance;
    private Roller roller;
    private Arm arm;
    private Swerve swerve;

    public static RobotContainer getInstance() {
        return instance;
    }

    public RobotContainer() {
        instance = this;
        roller = new Roller();
        arm = new Arm();
        swerve = new Swerve();

        OperatorControl operatorControl = new OperatorControl(roller, arm, swerve);

        roller.setDefaultCommand(
                operatorControl);
    }

}