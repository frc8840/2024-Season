package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights {
    Spark blinkIn;

    public Lights() {
        blinkIn = new Spark(0);
    }

    public void turnBlue() {
        blinkIn.set(0.87);
    }

    public void turnRed() {
        blinkIn.set(0.61);
    }

    public void turnGreen() {
        blinkIn.set(0.77);
    }

}
