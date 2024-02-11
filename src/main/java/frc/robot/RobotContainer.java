package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Swerve;

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

        OperatorControl operatorControl = new OperatorControl(roller, arm);
        DriverControl driverControl = new DriverControl(swerve);

        roller.setDefaultCommand(
                operatorControl);

        swerve.setDefaultCommand(
                driverControl);
    }

    public Swerve getSwerve() {
        return swerve;
    }
}