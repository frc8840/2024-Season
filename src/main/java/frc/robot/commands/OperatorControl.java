package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.PickUpNote;
import frc.team_8840_lib.info.console.Logger;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.subsystems.ArmShooter;

public class OperatorControl extends Command {

    private PS4Controller ps4controller;

    private Climber climber;
    private PickUpNote intake;
    private ArmShooter shooter;
    private Arm arm;

    private final Arm.ArmPosition[] heightOrder = new ArmPosition[] { ArmPosition.WRIST, ArmPosition.AMPSHOOTING,
            ArmPosition.SPEAKERSHOOTING };

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public OperatorControl(Arm arm, Climber climber, PickUpNote pIntake, ArmShooter shooter) {
        addRequirements(climber);
        this.climber = climber;
        this.intake = pIntake;
        this.shooter = shooter;
        this.arm = arm;

        ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);

    }

    @Override
    public void execute() {

        if (ps4controller.getTriangleButton()) {
            arm.setArmPosition(ArmPosition.AMPSHOOTING);
        }

        if (ps4controller.getL1ButtonPressed()) {
            arm.setArmPosition(ArmPosition.WRIST);
        }

        if (ps4controller.getR1ButtonPressed()) {
            arm.setArmPosition(ArmPosition.REST);
        }

        if (ps4controller.getSquareButtonPressed()) {
            arm.setArmPosition(ArmPosition.SPEAKERSHOOTING);
        }

        if (ps4controller.getCrossButtonPressed()) {
            climber.climb();
            // Logger.Log("climbing now");
        } else if (ps4controller.getCircleButtonPressed()) {
            climber.drop();
            // Logger.Log("dropping now");
        }

        if (ps4controller.getR2Button()) {
            intake.intake();
        } else if (ps4controller.getL2Button()) {
            intake.outtake();
        } else if (!intake.inComplexAction) {
            // not in the middle of complex action
            intake.stop();
        }

        if (ps4controller.getShareButtonPressed()) {
            arm.relax();
        }

        if (ps4controller.getOptionsButtonPressed()) {
            arm.gethard();
            shooter.gethard();
        }

        // the idea here is to run the shooter fo 500ms
        // to get it up to speed, then run the intake for 1000ms
        // then top both of them
        if (ps4controller.getTouchpadPressed()) {
            intake.inComplexAction = true;
            Command c = new SequentialCommandGroup(
                    new InstantCommand(() -> shooter.shoot()), // run the shooter
                    new WaitCommand(2),
                    new InstantCommand(() -> intake.intake()), // run the intake
                    new WaitCommand(1),
                    new InstantCommand(() -> {
                        shooter.stop();
                        intake.stop();
                    })); // stop them both
            c.schedule(); // make it happen!
        }
    }

}
