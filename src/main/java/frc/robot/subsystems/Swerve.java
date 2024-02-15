package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team_8840_lib.controllers.SwerveDrive;
import frc.team_8840_lib.utils.controllers.Pigeon;
import frc.team_8840_lib.utils.controllers.swerve.ModuleConfig;
import frc.team_8840_lib.utils.controllers.swerve.SwerveSettings;
import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;
import frc.team_8840_lib.utils.math.units.Unit;

public class Swerve extends SubsystemBase {
    public SwerveDrive swerveDrive;

    public Swerve() {
        SwerveSettings settings = new SwerveSettings();

        settings.maxSpeed = new Unit(4.5, Unit.Type.FEET);
        settings.trackWidth = new Unit(18.75, Unit.Type.INCHES);
        settings.wheelBase = new Unit(18.75, Unit.Type.INCHES);

        settings.invertGyro = true;
        settings.canCoderInverted = true;

        settings.drivePID = new PIDStruct(0.025, 0, 0, 0);
        settings.turnPID = new PIDStruct(0.012, 0, 0, 0);

        settings.updateKinematics();

        // this is specifically for joystick input/not cause issues w/ the motors:
        settings.threshold = 0.01;
        settings.useThresholdAsPercentage = true;

        final ModuleConfig frontLeft = new ModuleConfig(11, 12, 23, 105.8203);
        final ModuleConfig frontRight = new ModuleConfig(18, 17, 22, 323.877);
        final ModuleConfig backRight = new ModuleConfig(16, 15, 21, 41.8359);
        final ModuleConfig backLeft = new ModuleConfig(13, 14, 24, 215.332);

        swerveDrive = new SwerveDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                new Pigeon(Pigeon.Type.TWO, 42), // pigeon is a gyro (at CAN ID 42)
                settings // settings from before
        );

    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
