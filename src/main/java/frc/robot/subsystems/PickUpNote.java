package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.team_8840_lib.info.console.Logger;

public class PickUpNote extends SubsystemBase {

    private CANSparkMax iMotor;
    public boolean inComplexAction = false;
    // public long motorStartTime = -1; // not running

    public PickUpNote() {

        iMotor = new CANSparkMax(Settings.INTAKE_MOTOR_ID, MotorType.kBrushless);

        iMotor.restoreFactoryDefaults();
        iMotor.setIdleMode(IdleMode.kCoast);

        iMotor.setSmartCurrentLimit(80, 80);
        iMotor.setSecondaryCurrentLimit(85);

        iMotor.setOpenLoopRampRate(0.2);

        iMotor.setCANTimeout(20);

        iMotor.burnFlash();

    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Amperage", iMotor.getOutputCurrent());
        boolean noteIn = (Math.abs(iMotor.getOutputCurrent()) > 100);
        SmartDashboard.putBoolean("Intake Successful", noteIn);
    }
    

    public void intake() {
        iMotor.set(Settings.PICKUP_INTAKE_SPEED);
        Logger.Log("Intake Motor Amperage: " + iMotor.getOutputCurrent());
        // if (motorStartTime < 0) {
        // motorStartTime = System.currentTimeMillis();
        // }
    }

    public void outtake() {
        iMotor.set(Settings.PICKUP_OUTTAKE_SPEED);
        Logger.Log("Outtake Motor Amperage: " + iMotor.getOutputCurrent());
    }

    public void stop() {
        iMotor.set(0);
        inComplexAction = false;
        // motorStartTime = -1; // stopped
    }

    public double getAmperage() {
        return iMotor.getOutputCurrent();
    }

    // public long getTimeRunning() {
    // if (motorStartTime < 0)
    // return 0;
    // return System.currentTimeMillis() - motorStartTime;
    // }

}
