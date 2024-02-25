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

public class DriverControl extends Command {

    private XboxController xboxcontroller;
    private NewSwerve swerve;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public DriverControl(NewSwerve swerve) {
        addRequirements(swerve);

        this.swerve = swerve;

        xboxcontroller = new XboxController(Settings.OPERATOR_CONTROLLER_PORT);
    }

    @Override
    public void execute() {

        // If user pressed the 'A' key, then we print the CANCoder angles
        if (xboxcontroller.getAButtonPressed()) {
            NewSwerveModule[] modules = swerve.getModules();
            SwerveModulePosition[] positions = swerve.getPositions();
            Logger.Log("CAN Angles:");
            for (int i = 0; i < 4; i++) {
                Logger.Log(modules[i].toString() + ": " + modules[i].getCanCoderAngle().getDegrees());
            }
        }
        // If user pressed the 'B' key, then we print the motor encoder angles
        if (xboxcontroller.getBButtonPressed()) {
            NewSwerveModule[] modules = swerve.getModules();
            SwerveModulePosition[] positions = swerve.getPositions();
            Logger.Log("Motor Angles:");
            for (int i = 0; i < 4; i++) {
                Logger.Log(modules[i].toString() + ": " + positions[i].angle);
            }
        }
        // If user pressed the 'X' key, then we run the motors straight
        if (xboxcontroller.getXButton()) {
            Logger.Log("Drive straight:");
            swerve.setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
            }, false);
        } else {

            /* Get Values, Deadband */
            double translationVal = translationLimiter.calculate(
                    MathUtil.applyDeadband(xboxcontroller.getRightY(), Constants.Swerve.stickDeadband));
            double strafeVal = strafeLimiter.calculate(
                    MathUtil.applyDeadbanxboxcontroller.getRightX(), Constants.Swerve.stickDeadband));
            double rotationVal = rotationLimiter.calculate(
                    MathUtil.applyDeadband(xboxcontroller.getLeftX(), Constants.Swerve.stickDeadband));

            /* Drive */
            swerve.drive(
                    new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                    rotationVal * Constants.Swerve.maxAngularVelocity,
                    false,
                    false);
        }

    }

}
