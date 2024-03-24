package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Settings;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Arm.ArmPosition;

public class DriverControl extends Command {

    private XboxController xboxcontroller;
    private NewSwerve swerve;
    private Arm arm;
    private Lights lights;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(10);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(10);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(10);

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public DriverControl(NewSwerve swerve, Arm arm, Lights lights) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.arm = arm;
        this.lights = lights;
        xboxcontroller = new XboxController(Settings.DRIVER_CONTROLLER_PORT);
    }

    @Override
    public void execute() {

        if (xboxcontroller.getXButtonPressed()) {
            swerve.zeroGyro();
        }

        // get values from the Xbox Controller joysticks
        // apply the deadband so we don't do anything right around the center of the
        // joysticks
        double translationVal = translationLimiter.calculate(
                MathUtil.applyDeadband(xboxcontroller.getLeftY(), 0.05));
        double strafeVal = strafeLimiter.calculate(
                MathUtil.applyDeadband(-xboxcontroller.getLeftX(), 0.05));
        double rotationVal = rotationLimiter.calculate(
                MathUtil.applyDeadband(-xboxcontroller.getRightX(), 0.05));

        /* Drive */
        swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity,
                true);
    }

}
