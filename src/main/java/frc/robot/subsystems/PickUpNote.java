package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.team_8840_lib.info.console.Logger;

public class PickUpNote extends SubsystemBase {

    private CANSparkMax iMotor;

    public PickUpNote() {

        iMotor = new CANSparkMax(Settings.INTAKE_MOTOR_ID, MotorType.kBrushless);

        iMotor.restoreFactoryDefaults();

        iMotor.setSmartCurrentLimit(80, 80);
        iMotor.setSecondaryCurrentLimit(85);

        iMotor.setOpenLoopRampRate(0.2);

        iMotor.setCANTimeout(20);

        iMotor.burnFlash();

    }

    public void pIntake() {
        iMotor.set(Settings.INTAKE_SPEED);
        Logger.Log("Intake Motor Amperage: " + iMotor.getOutputCurrent());
    }

    public void pOuttake() {
        iMotor.set(Settings.OUTTAKE_SPEED);
        Logger.Log("Outtake Motor Amperage: " + iMotor.getOutputCurrent());
    }

    public void pStop() {
        iMotor.set(0);
    }

}
