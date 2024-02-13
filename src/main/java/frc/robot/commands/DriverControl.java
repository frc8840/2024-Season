package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Swerve;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.math.units.Unit;

public class DriverControl extends Command {

    private XboxController xboxcontoller;
    private Swerve swerve;
    private Roller roller;

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public DriverControl(Swerve swerve, Roller roller) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.roller = roller;

        xboxcontoller = new XboxController(Settings.OPERATOR_CONTROLLER_PORT);
    }

    @Override
    public void execute() {

        int n = 0;
        for (SwerveModulePosition position : swerve.getSwerveDrive().getSwervePositions()) {
            SmartDashboard.putNumber("Swerve-Mod" + n, position.distanceMeters);
            n++;
        }
        SmartDashboard.updateValues();

        if (Math.abs(getForward()) < 0.1 && Math.abs(getStrafe()) < 0.1) {
            if (Math.abs(xboxcontoller.getRightX()) < 0.1) {
                swerve.swerveDrive.stop();
            } else {
                // If the rotate threshold is met, rotate the robot
                swerve.swerveDrive.spin(Rotation2d.fromRadians(xboxcontoller.getRightX()), Robot.isReal());
            }
            return;
        }

        // Create a new Translation2d with the x and y values of the controller.
        Translation2d translation = new Translation2d(
                getForward(), // forward from the controller
                getStrafe() // strafe from the controller
        );

        // Multiply by the max speed.
        translation = translation.times(swerve.swerveDrive.getSettings().maxSpeed.get(Unit.Type.METERS));

        // Drive
        swerve.swerveDrive.drive(translation, Rotation2d.fromRadians(xboxcontoller.getRightX()), true, Robot.isReal());

        // THIS IS TEST FOR MOTOR
        if (xboxcontoller.getAButton()) {
            roller.intake();
        } else if (xboxcontoller.getYButton()) {
            roller.outtake(true);
        } else {
            roller.stop();
        }
    }

    public double getForward() {
        return -xboxcontoller.getLeftY();
    }

    public double getStrafe() {
        return xboxcontoller.getLeftX();
    }

}
