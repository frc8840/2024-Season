package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
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
    private ArmShooter outtake;
    // private Arm arm;

    private final Arm.ArmPosition[] heightOrder = new ArmPosition[] { ArmPosition.HYBRID, ArmPosition.MID_CONE,
            ArmPosition.HIGH_CONE };
    private int selectedPosition = 0; // The selected index of the height order, changed through the arrow keys on the
                                      // PS4 controller.
    private boolean armInPosition = false;

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public OperatorControl(Climber climber, PickUpNote pIntake, ArmShooter outtake) {
        addRequirements(climber);
        this.climber = climber;
        this.intake = pIntake;
        this.outtake = outtake;
        // this.arm = arm;

        ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);

    }

    @Override
    public void execute() {

        Logger.Log("L position: " + climber.lEncoder.getPosition());
        Logger.Log("R position: " + climber.rEncoder.getPosition());

        if (ps4controller.getL2Button()) {
            climber.Lintake();
            climber.Rintake();
        } else if (ps4controller.getL1Button()) {
            climber.Louttake();
            climber.Routtake();
        } else {
            climber.leftStop();
            climber.rightStop();
        }

        if (ps4controller.getR2ButtonPressed()) {
            climber.climb();
        } else if (ps4controller.getR1ButtonPressed()) {
            climber.drop();
        }
        // poop
        if (ps4controller.getCircleButton()) {
            intake.pIntake();
        } else if (ps4controller.getCircleButton()) {
            intake.pOuttake();
        } else {
            intake.pStop();
        }

        if (ps4controller.getSquareButton()) {
            outtake.sIntake();
        } else if (ps4controller.getSquareButton()) {
            outtake.sOuttake();
        } else {
            outtake.sStop();
        }

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
