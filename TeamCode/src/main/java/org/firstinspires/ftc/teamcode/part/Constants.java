package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config("CONSTANTS")
public class Constants {
    // Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // Turret
    public static double TURRET_PID_VISION_P = -2.5;
    public static double TURRET_PID_VISION_I = 0;
    public static double TURRET_PID_VISION_D = 0;
    public static double TURRET_PID_THRESHOLD = 1.0;

    public static double TURRET_PID_POSITION_P = -0.001;
    public static double TURRET_PID_POSITION_I = 0;
    public static double TURRET_PID_POSITION_D = 0.001;
    public static double TURRET_TARGET_POS = 0;
    public static double TURRET_LEFT_LIMIT = -1100;
    public static double TURRET_RIGHT_LIMIT = 950;
    public static double TURRET_LEFT_END = -1400;
    public static double TURRET_RIGHT_END = 1200;
    public static double TURRET_DIFF_THRESHOLD = 10;

    public static int TURRET_ONE_REV_TICKS = 1900;
    public static double TURRET_MAXIMUM_POWER = 0.6;

    // Drive
    public static double DRIVE_POWER = 1;

    // Shooter
    public static double AAA_SHOOTER_VELOCITY = 5; // m/s;
    public static double AAA_SHOOTER_TEST_ANGLE = 0;

    public static final double SHOOTER_WHEEL_RADIUS = 0.05; //m
    public static final double SHOOTER_GEAR_RATIO = 3;

    public static boolean SHOOTER_RUNNING = true;

    public static double SHOOTER_STOPPER_OPEN_ANGLE = 0.8;
    public static double SHOOTER_STOPPER_CLOSE_ANGLE = 0.65;


    // Vision
    public enum PIPELINE {
        OBELISK_DETECTION,
        RED_GOAL,
        BLUE_GOAL
    };
    public static int RED_PIPELINE = 4;
    public static int BLUE_PIPELINE = 5;
    public static int OBELISK_PIPELINE = 6;



}
