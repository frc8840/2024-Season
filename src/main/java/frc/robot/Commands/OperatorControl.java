package frc.robot.Commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Arm.ArmPosition;
import frc.team_8840_lib.listeners.Robot;
import frc.robot.Subsystems.Roller;
import frc.robot.Subsystems.Swerve;
import frc.team_8840_lib.utils.math.units.Unit;

public class OperatorControl extends Command {

    private PS4Controller ps4controller;
    private XboxController xboxcontoller;

    private Roller roller;
    private Arm arm;
    private Swerve swerve;

    private final Arm.ArmPosition[] heightOrder = new ArmPosition[] { ArmPosition.HYBRID, ArmPosition.MID_CONE,
            ArmPosition.HIGH_CONE };
    private int selectedPosition = 0; // The selected index of the height order, changed through the arrow keys on the
                                      // PS4 controller.
    private boolean armInPosition = false;

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public OperatorControl(Roller roller, Arm arm, Swerve swerve) {
        addRequirements(roller);
        this.roller = roller;
        this.arm = arm;
        this.swerve = swerve;
        ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);
        xboxcontoller = new XboxController(Settings.OPERATOR_CONTROLLER_PORT);
    }

    @Override
    public void execute() {
        if (ps4controller.getL2Button()) {
            roller.intake();
        } else if (ps4controller.getR2Button() || ps4controller.getR1Button()) {
            roller.outtake(ps4controller.getR2Button());
        } else {
            roller.stop();
        }

        if (ps4controller.getPOV() == 270) {
            selectedPosition--;
            if (selectedPosition < 0) {
                selectedPosition = heightOrder.length - 1;
            }
        } else if (ps4controller.getPOV() == 90) {
            selectedPosition++;
            if (selectedPosition >= heightOrder.length) {
                selectedPosition = 0;
            }
        }

        if (ps4controller.getCircleButtonReleased()) {
            armInPosition = !armInPosition;

            if (armInPosition) {
                arm.setArmPosition(heightOrder[selectedPosition]);
            } else {
                arm.setArmPosition(ArmPosition.REST);
            }
        } else if (ps4controller.getCrossButtonReleased() && !armInPosition) {
            arm.setArmPosition(ArmPosition.DOUBLE_SUBSTATION_INTAKE);

            armInPosition = true;
        }

        arm.reportToNetworkTables();

        SmartDashboard.putString("Selected Position", heightOrder[selectedPosition].name());

        // ...

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

    }

    public double getForward() {
        return -xboxcontoller.getLeftY();
    }

    public double getStrafe() {
        return xboxcontoller.getLeftX();
    }

}
