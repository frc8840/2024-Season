package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Settings;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmPosition;

public class OperatorControl extends Command {

    private PS4Controller ps4controller;

    private Shooter roller;
    // private Arm arm;

    private final Arm.ArmPosition[] heightOrder = new ArmPosition[] { ArmPosition.HYBRID, ArmPosition.MID_CONE,
            ArmPosition.HIGH_CONE };
    private int selectedPosition = 0; // The selected index of the height order, changed through the arrow keys on the
                                      // PS4 controller.
    private boolean armInPosition = false;

    // Make sure the roller imported is the one from subsystems! Not from settings.
    public OperatorControl(Shooter roller) {
        addRequirements(roller);
        this.roller = roller;
        // this.arm = arm;

        ps4controller = new PS4Controller(Settings.OPERATOR_CONTROLLER_PORT);

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
