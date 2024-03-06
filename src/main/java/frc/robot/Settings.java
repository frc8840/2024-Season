package frc.robot;

import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class Settings {

    // CLIMBER SETTINGS
    public static final int LCLIMBER_MOTOR_ID = 30;
    public static final int RCLIMBER_MOTOR_ID = 31;
    public static final double CLIMBER_OUTTAKE_SPEED = 0.5;
    public static final double CLIMBER_INTAKE_SPEED = -0.5;

    // CONTROLLER SETTINGS
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // ARM SETTINGS
    public static final int SHOULDER_MOTOR_ID = 40;
    public static final int ELBOW_MOTOR_ID = 41;
    public static final int WRIST_MOTOR_ID = 42;

    public static final double SHOULDER_GEAR_RATIO = 266 / 1;
    public static final double ELBOW_GEAR_RATIO = 266 / 1;
    public static final double WRIST_GEAR_RATIO = 128 / 1;

    public static final PIDStruct SHOULDER_PID = new PIDStruct(0.10, 0.0, 40.0);
    public static final PIDStruct ELBOW_PID = new PIDStruct(0.10, 0.0, 40.0);
    public static final PIDStruct WRIST_PID = new PIDStruct(0.10, 0.0, 40.0);

    public static final double MAX_SHOULDER_SPEED = 0.8;
    public static final double MAX_ELBOW_SPEED = 0.8;
    public static final double MAX_WRIST_SPEED = 0.8;

    public static final double CLOSED_LOOP_RAMP_RATE = 1.0;

    // INTAKE SETTINGS
    public static final int INTAKE_MOTOR_ID = 50;
    public static final double PICKUP_OUTTAKE_SPEED = -1.0;
    public static final double PICKUP_INTAKE_SPEED = 1.0;

    // SHOOTER SETTINGS
    public static final int SHOOTER_MOTOR_ID = 51;
    public static final int SHOOTER_MOTOR_ID2 = 52;
    public static final double SHOOTER_OUT_SPEED = 1.0;
    public static final double SHOOTER_IN_SPEED = -1.0;
}
