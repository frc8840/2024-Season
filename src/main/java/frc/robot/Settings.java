package frc.robot;

import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class Settings {

    // ROLLER SETTINGS
    public static final int ROLLER_MOTOR_ID = 30;
    public static final double FAST_OUTTAKE_SPEED = 0.7;
    public static final double SLOW_OUTTAKE_SPEED = 0.1;
    public static final double INTAKE_SPEED = -0.7;

    // CONTROLLER SETTINGS
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // ARM SETTINGS
    public static final int BASE_MOTOR_ID = 31;
    public static final int ELBOW_MOTOR_ID = 32;

    public static final double GEAR_RATIO = 192 / 1;

    public static final PIDStruct BASE_PID = new PIDStruct(0.010, 0.0, 0.0);
    public static final PIDStruct ELBOW_PID = new PIDStruct(0.010, 0.0, 0.0);

    public static final double MAX_BASE_SPEED = 0.8;
    public static final double MAX_ELBOW_SPEED = 0.8;

    public static final double CLOSED_LOOP_RAMP_RATE = 1.0;
}
