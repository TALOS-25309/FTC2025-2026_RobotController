package org.firstinspires.ftc.teamcode.part;

import com.acmerobotics.dashboard.config.Config;

@Config("CONSTANTS")
public class Constants {
    // Drive
    public static double DRIVE_POWER = 1;

    // Intake
    public static double INTAKE_POWER = 1;
    public static double INTAKE_REVERSE_POWER = 0.4;

    // Turret
    public static double TURRET_PID_VISION_P = -0.015;
    public static double TURRET_PID_VISION_I = 0;
    public static double TURRET_PID_VISION_D = 0;
    public static double TURRET_PID_THRESHOLD = 0.001;

    public static double TURRET_PID_POSITION_P = -0.001;
    public static double TURRET_PID_POSITION_I = 0;
    public static double TURRET_PID_POSITION_D = 0.001;

    public static double TURRET_LEFT_END = -600;
    public static double TURRET_RIGHT_END = 600;

    public static int TURRET_ONE_REV_TICKS = 1900;
    public static double TURRET_MAXIMUM_POWER = 0.6;
    public static double TURRET_ROTATION_VEL = 4;
    public static double TURRET_ROTATION_VEL_FASTER = 20;


    // Shooter
    public static double AAA_SHOOTER_VELOCITY = 6; // m/s;
    public static double AAA_SHOOTER_TEST_ANGLE = 0.7;

    public static boolean SHOOTER_RUNNING = true;

    public static double SHOOTER_STOPPER_OPEN_ANGLE = 0.53;
    public static double SHOOTER_STOPPER_CLOSE_ANGLE = 0.63;

    public static double SHOOTER_TIME_INTERVAL_ONE = 0.3; // s
    public static double SHOOTER_TIME_INTERVAL_TWO = 0.3; // s

    public static double SHOOTER_TIME_SHOOT_ONE = 0.3;
    public static double SHOOTER_TIME_SHOOT_TWO = 0.6;
    public static double SHOOTER_TIME_SHOOT_THREE = 0.9;

    public static double SHOOTER_TIME_DELAY = 1.0;


    public static double SHOOTER_UPPER_PID_F = 12;
    public static double SHOOTER_LOWER_PID_F = 11.57;

    // 1-1 매칭
    public static double[] SHOOTER_DISTANCES = {1.0, 1.3, 1.7}; // m
    public static double[] SHOOTER_ANGLES    = {0.63, 0.56, 0.5}; // servo_angle

    public static double GOAL_X = 72.0; // 예: FTC 필드 끝
    public static double GOAL_Y = 36.0; // 예: 중앙


    public static double DRIVE_ROTATION_SPEED = 0.5;


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
