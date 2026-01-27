package org.firstinspires.ftc.teamcode.opmode.auto;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config("Turret")
public class AutoTurret {
    // Note the motor and the encoder are both connected to the same port,
    // thus accessible by this class.
    private final DcMotorEx motor;

    // PID controls
    PID pidController;

    // Vision
    private Vision limelight;

    // Interface
    // Terrible naming, but thi will keep it sorted at the web ui.
    public static double pid_P = 0.0;
    public static double pid_I = 0.0;
    public static double pid_D = 0.0;
    public static int runToTarget_ControlThres = 10;
    public static int runToTarget_HardLimits = 960;
    public static int runToTarget_oneEightyTurn = 950;
    public static double runToTarget_power = 1;

    AutoTurret(DcMotorEx motor, Vision limelight) {
        this.motor = motor;
        this.limelight = limelight;
    }

    void init() {
        pidController = new PID(pid_P, pid_I, pid_D);
        return;
    }

    void start() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // This is our zero from now on.
    }

    /**
     * Looks at a position.
     * @param ticks Measured in ticks. CW is +, CCW is -. Zero point is straight front of the robot.
     * @return input validity.
     */
    boolean gaze_tick(int ticks) {
        if (ticks > runToTarget_HardLimits || ticks < -1 * runToTarget_HardLimits)
            return false;
        motor.setPower(1);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    /**
     * Wrapper for gaze tick, just in radians.
     * @param radians Angle in radians. Zero point is straight front, CW+, CCW-
     * @return Doesn't. Input is always valid.
     */
    void gaze(double radians) {
        radians = Math.atan2(Math.sin(radians), Math.cos(radians));
        long ticks = Math.round(radians * (Math.PI / (double)runToTarget_oneEightyTurn));
        gaze_tick((int)ticks);
        return;
    }

    boolean visionAlign(int timeoutMillis) {
        return false;
    }
}
