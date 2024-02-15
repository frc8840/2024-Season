package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.math.units.Unit;

public class DriverControl extends Command {

    private XboxController xboxcontroller;
    private Swerve swerve;
    private Shooter roller;

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public DriverControl(Swerve swerve, Shooter roller) {
        addRequirements(swerve);

        this.swerve = swerve;
        this.roller = roller;

        xboxcontroller = new XboxController(Settings.OPERATOR_CONTROLLER_PORT);
    }

    @Override
    public void execute() {

        int n = 0;
        for (SwerveModulePosition position : swerve.getSwerveDrive().getSwervePositions()) {
            SmartDashboard.putNumber("Swerve-Mod" + n, position.distanceMeters);
            n++;
        }
        SmartDashboard.updateValues();

        // if we left joystick is close to the center
        if (Math.abs(-xboxcontroller.getLeftY()) < 0.1 && Math.abs(xboxcontroller.getLeftX()) < 0.1) {
            // if the right joystick is close the center
            if (Math.abs(xboxcontroller.getRightX()) < 0.1) {
                swerve.swerveDrive.stop();
            } else {
                // If the right joystick threshold is met, rotate the robot
                swerve.swerveDrive.spin(Rotation2d.fromRadians(xboxcontroller.getRightX()),
                        Robot.isReal());
            }
            return;
        }
        // otherwise left joystick is not in the center

        // Create a new Translation2d with the desired direction and speed
        Translation2d translation = new Translation2d(
                -xboxcontroller.getLeftY(), // forward from the controller
                xboxcontroller.getLeftX() // strafe from the controller
        );

        // Scale by the max speed.
        translation = translation.times(swerve.swerveDrive.getSettings().maxSpeed.get(Unit.Type.METERS));

        // Tell the swerve drive to go in that direction
        swerve.swerveDrive.drive(translation,
                Rotation2d.fromRadians(xboxcontroller.getRightX()), true, Robot.isReal());

        // THIS IS TEST FOR MOTOR

        // if (xboxcontroller.getAButton()) {
        // roller.intake();
        // Logger.Log("xbox A button was pressed");
        // } else if (xboxcontroller.getYButton()) {
        // roller.outtake(true);
        // Logger.Log("xbox Y button was pressed");
        // } else {
        // roller.stop();
        // }
    }

}
