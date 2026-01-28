package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config("AutoTurret")
public class AutoTurret {
    private final DcMotorEx motor;

    // PID controls
    PID pidController;

    // Vision
    private Vision limelight;
    boolean visionActive = false;
    private long visionStartTime;

    // --- NEW: target tracking ---
    private int targetTicks = 0;

    // Interface
    public static double pid_P = 0.02;
    public static double pid_I = 0.0;
    public static double pid_D = 0.0;
    public static int runToTarget_HardLimit = 960;
    public static int runToTarget_TICKS_PER_180_DEG = 950;
    public static double runToTarget_power = 1;
    public static double TURRET_PID_THRESHOLD = 1;
    public static double TURRET_MAXIMUM_POWER = 0.6;
    private static int TURRET_TIMEOUT = 1500;

    AutoTurret(DcMotorEx motor, Vision limelight) {
        this.motor = motor;
        this.limelight = limelight;
        pidController = new PID(pid_P, pid_I, pid_D);
    }

    void init() {}

    void start() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Looks at a position.
     * @param ticks CW+, CCW-, 0 = forward
     */
    boolean gaze_tick(int ticks) {
        if (visionActive) return false;
        if (Math.abs(ticks) > runToTarget_HardLimit) return false;

        targetTicks = ticks; // <-- store target

        motor.setPower(runToTarget_power);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    /**
     * Wrapper for gaze_tick, radians.
     */
    boolean gaze(double radians) {
        if (visionActive) return false;

        radians = Math.atan2(Math.sin(radians), Math.cos(radians));
        int ticks = (int) Math.round(
                radians * (runToTarget_TICKS_PER_180_DEG / Math.PI)
        );
        return gaze_tick(ticks);
    }

    /**
     * Turns the turret relative to its current position.
     * @param radians CW+, CCW-
     * @return true if command accepted
     */
    boolean turn(double radians) {
        if (visionActive) return false;

        // Normalize radians to [-pi, pi]
        radians = Math.atan2(Math.sin(radians), Math.cos(radians));

        int deltaTicks = (int) Math.round(
                radians * (runToTarget_TICKS_PER_180_DEG / Math.PI)
        );

        int currentTicks = motor.getCurrentPosition();
        int target = currentTicks + deltaTicks;

        // Respect hard limits
        if (Math.abs(target) > runToTarget_HardLimit) return false;

        return gaze_tick(target);
    }


    // -------------------------------
    // Vision
    // -------------------------------

    void startVisionAlign() {
        visionStartTime = System.currentTimeMillis();
        visionActive = true;
        pidController.resetIntegral();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void visionUpdate() {
        limelight.update();
        if (!visionActive) return;

        if (System.currentTimeMillis() - visionStartTime > TURRET_TIMEOUT) {
            motor.setPower(0);
            visionActive = false;
            return;
        }

        boolean detected = limelight.tagDetected();
        double current_angle = limelight.getAngle();
        double error = -current_angle;

        TelemetrySystem.addClassData("AutoTurret", "Det", detected);
        TelemetrySystem.addClassData("AutoTurret", "Error", error);
        TelemetrySystem.update();

        if (Math.abs(error) < TURRET_PID_THRESHOLD) {
            motor.setPower(0);
            visionActive = false;
            return;
        }

        double pidOutput = pidController.update(
                error,
                -TURRET_MAXIMUM_POWER,
                TURRET_MAXIMUM_POWER
        );
        motor.setPower(pidOutput);
    }

    void blockingAlign() {
        startVisionAlign();
        while (visionActive) {
            update();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }

    void constantsUpdate() {
        pidController.updatePID(pid_P, pid_I, pid_D);
    }

    void update() {
        visionUpdate();
        constantsUpdate();
    }

    boolean isBusy() {
        return visionActive ||
                (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION && motor.isBusy());
    }
}