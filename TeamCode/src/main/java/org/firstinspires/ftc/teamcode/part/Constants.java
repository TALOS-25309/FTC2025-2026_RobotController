package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config("CONSTANTS")
public class Constants {
    // Intake
    public static double INTAKE_POWER = 0.6;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // TURRET
    public static double P = -2.5;
    public static double I = 0;
    public static double D = 1;

    public static double TURRET_LEFT_LIMIT = 0;
    public static double TURRET_RIGHT_LIMIT = 0;
    public static double TURRET_LEFT_END = 0;
    public static double TURRET_RIGHT_END = 0;

    public static int ONE_REV_TICKS = 0;
    public static double TURRET_MAXIMUM_POWER = 0.6;

    // DRIVE
    public static double DRIVE_POWER = 1;

    // SHOOTER
    public static double SHOOTER_POWER = 0.5;
    public static boolean SHOOTER_RUNNING = false;

    public static double STOPPER_OPEN_ANGLE = 0;
    public static double STOPPER_CLOSE_ANGLE = 0.22;
    public static double STOPPER_TEST_ANGLE = 0;
    public static double SHOOTER_DOWN_ANGLE = 0.6;
    public static double SHOOTER_UP_ANGLE = 0;


}
