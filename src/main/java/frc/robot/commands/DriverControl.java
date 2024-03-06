package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Settings;
import frc.robot.subsystems.NewSwerve;
import frc.robot.subsystems.NewSwerveModule;
import frc.team_8840_lib.info.console.Logger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.Robot;

public class DriverControl extends Command {

    private XboxController xboxcontroller;
    private NewSwerve swerve;
    private Arm arm;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public DriverControl(NewSwerve swerve, Arm arm) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.arm = arm;

        xboxcontroller = new XboxController(Settings.DRIVER_CONTROLLER_PORT);
    }

    @Override
    public void execute() {

        if (xboxcontroller.getXButtonPressed()) {
            swerve.zeroGyro();
        } else if (xboxcontroller.getAButtonPressed()) {
            arm.setArmPosition(ArmPosition.INTAKEDEMO);
        } else {

            // get values from the Xbox Controller joysticks
            // apply the deadband so we don't do anything right around the center of the
            // joysticks
            double translationVal = translationLimiter.calculate(
                    MathUtil.applyDeadband(xboxcontroller.getLeftY(), Constants.Swerve.stickDeadband));
            double strafeVal = strafeLimiter.calculate(
                    MathUtil.applyDeadband(-xboxcontroller.getLeftX(), Constants.Swerve.stickDeadband));
            double rotationVal = rotationLimiter.calculate(
                    MathUtil.applyDeadband(-xboxcontroller.getRightX(), Constants.Swerve.stickDeadband));

            /* Drive */
            swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    true,
                    false);
        }

    }

}
