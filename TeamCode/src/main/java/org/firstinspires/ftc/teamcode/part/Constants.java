package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config("CONSTANTS")
public class Constants {
    // Intake
    public static double INTAKE_POWER = 0.6;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // TURRET
    public static double TURRET_PID_P = -2.5;
    public static double TURRET_PID_I = 0;
    public static double TURRET_PID_D = 2;

    public static double TURRET_LEFT_LIMIT = 0;
    public static double TURRET_RIGHT_LIMIT = 8000;
    public static double TURRET_LEFT_END = 0;
    public static double TURRET_RIGHT_END = 0;

    public static int TURRET_ONE_REV_TICKS = 0;
    public static double TURRET_MAXIMUM_POWER = 0.6;

    // DRIVE
    public static double DRIVE_POWER = 1;

    // SHOOTER
    public static double AAA_SHOOTER_VELOCITY = 0.5; // m/s;
    public static int AAA_SHOOTER_RPM = 100;
    public static double SHOOTER_WHEEL_RADIUS = 0.1; //m
    public static double SHOOTER_GEAR_RATIO = 3;

    public static boolean SHOOTER_RUNNING = true;

    public static double SHOOTER_STOPPER_OPEN_ANGLE = 0.4;
    public static double SHOOTER_STOPPER_CLOSE_ANGLE = 0.55;

    public static double SHOOTER_DOWN_ANGLE = 0.6;
    public static double SHOOTER_UP_ANGLE = 0;
    public static double AAA_SHOOTER_TEST_ANGLE = 0;




}
