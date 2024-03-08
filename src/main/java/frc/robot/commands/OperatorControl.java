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
    private int selectedPosition = 0; // The selected index of the height order, changed through the arrow keys on the
                                      // PS4 controller.
    private boolean armInPosition = false;

    private String lastButtonPressed = null;

    // these are for the shooting action
    long shooterStarted = -1;
    long intakeStarted = -1;

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

        // long now = System.currentTimeMillis();
        // if (shooterStarted + 2000 < now) {
        // // it has been 1000ms since shooter started
        // // stop this whole thing
        // shooterStarted = -1;
        // intakeStarted = -1;
        // shooter.stop();
        // intake.stop();
        // } else if (shooterStarted + 1000 < now && intakeStarted < 0) {
        // // it has been 500ms since shooter started and intake hasn't started yet
        // // start the intake
        // this.intake.intake();
        // intakeStarted = now;
        // }
        // this function is calld by WPILIB 50 times per second
        // if (ps4controller.getTriangleButtonPressed()) {
        // arm.setArmPosition(ArmPosition.SHOULDER);
        // }

        if (ps4controller.getL2ButtonPressed()) {
            arm.setArmPosition(ArmPosition.ELBOW);
        }

        if (ps4controller.getL1ButtonPressed()) {
            arm.setArmPosition(ArmPosition.WRIST);
        }

        if (ps4controller.getCrossButtonPressed()) {
            arm.setArmPosition(ArmPosition.REST);
        }

        if (ps4controller.getPSButtonPressed()) {
            arm.setArmPosition(ArmPosition.AMPSHOOTING);
        }

        if (ps4controller.getR2ButtonPressed()) {
            climber.climb();
            Logger.Log("climbing now");
        } else if (ps4controller.getR1ButtonPressed()) {
            climber.drop();
            Logger.Log("dropping now");
        }

        if (ps4controller.getCircleButton()) {
            intake.intake();
        } else if (ps4controller.getTouchpad()) {
            intake.outtake();
        } else if (intakeStarted < 0) {
            // not in the middle of complex action
            intake.stop();
        }

        if (ps4controller.getSquareButton()) {
            shooter.outtake();
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
        if (ps4controller.getTriangleButtonPressed()) {
            // shooter.outtake(); // start the shooter
            // shooterStarted = now;
            Command c = new SequentialCommandGroup(
                    new InstantCommand(() -> shooter.outtake()), // run the shooter
                    new WaitCommand(1),
                    new InstantCommand(() -> intake.intake()), // run the intake
                    new WaitCommand(1),
                    new InstantCommand(() -> {
                        shooter.stop();
                        intake.stop();
                    })); // stop them both
            c.schedule(); // make it happen!

        }
        // else if (shooterStarted < 0) {
        // // not in the middle of complex action
        // shooter.stop();
        // }

        // if (ps4controller.getPOV() == 270) {
        // selectedPosition--;
        // if (selectedPosition < 0) {
        // selectedPosition = heightOrder.length - 1;
        // }
        // } else if (ps4controller.getPOV() == 90) {
        // selectedPosition++;
        // if (selectedPosition >= heightOrder.length) {
        // selectedPosition = 0;
        // }
        // }

        /*
         * if (ps4controller.getCircleButtonReleased()) {
         * armInPosition = !armInPosition;
         * 
         * if (armInPosition) {
         * arm.setArmPosition(heightOrder[selectedPosition]);
         * } else {
         * arm.setArmPosition(ArmPosition.REST);
         * }
         * } else if (ps4controller.getCrossButtonReleased() && !armInPosition) {
         * arm.setArmPosition(ArmPosition.DOUBLE_SUBSTATION_INTAKE);
         * 
         * armInPosition = true;
         * }
         * 
         * arm.reportToNetworkTables();
         * 
         * SmartDashboard.putString("Selected Position",
         * heightOrder[selectedPosition].name());
         */
    }

}
