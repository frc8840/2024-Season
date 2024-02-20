package frc.robot;

import frc.robot.commands.DriverControl;
import frc.robot.commands.OperatorControl;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    private static RobotContainer instance;
    private Shooter shooter;
    private Swerve swerve;

    public static RobotContainer getInstance() {
        return instance;
    }

    public RobotContainer() {
        instance = this;
        // shooter = new Shooter();
        swerve = new Swerve();

        // OperatorControl operatorControl = new OperatorControl(shooter);
        DriverControl driverControl = new DriverControl(swerve);

        // shooter.setDefaultCommand(operatorControl);

        swerve.setDefaultCommand(driverControl);
    }

    public Swerve getSwerve() {
        return swerve;
    }
}