package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.feature.PID;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

import com.acmerobotics.dashboard.config.Config;

@Config("AutoTurret")
public class AutoTurret {
    // Note the motor and the encoder are both connected to the same port,
    // thus accessible by this class.
    private final DcMotorEx motor;

    // PID controls
    PID pidController;

    // Vision
    private Vision limelight;
    boolean visionActive = false;
    private long visionStartTime;

    // Interface
    // Terrible naming, but thi will keep it sorted at the web ui.
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

    void init() {
        return;
    }

    void start() {
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // This is our zero from now on.
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // This is our default. It does nothing, but why not
    }

    /**
     * Looks at a position.
     * @param ticks Measured in ticks. CW is +, CCW is -. Zero point is straight front of the robot.
     * @return Validity.
     */
    boolean gaze_tick(int ticks) {
        if (visionActive)
            return false;
        if (ticks > runToTarget_HardLimit || ticks < -1 * runToTarget_HardLimit)
            return false;
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(runToTarget_power);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        return true;
    }

    /**
     * Wrapper for gaze tick, just in radians.
     * @param radians Angle in radians. Zero point is straight front, CW+, CCW-
     * @return Validity
     */
    boolean gaze(double radians) {
        if (visionActive)
            return false;
        radians = Math.atan2(Math.sin(radians), Math.cos(radians));
        long ticks = Math.round(radians * (runToTarget_TICKS_PER_180_DEG / Math.PI));
        return gaze_tick((int)ticks);
    }

    void startVisionAlign() {
        visionStartTime = System.currentTimeMillis();
        visionActive = true;
        pidController.resetIntegral();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void visionUpdate() {
        if (!visionActive) return;

        if (System.currentTimeMillis() - visionStartTime > TURRET_TIMEOUT) {
            this.motor.setPower(0);
            visionActive = false;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return;
        }

        double current_angle = this.limelight.getAngle();
        double error = -current_angle;
        if (Math.abs(error) < TURRET_PID_THRESHOLD) {
            motor.setPower(0);
            visionActive = false;
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            return;
        }
        double pidOutput = pidController.update(error, -TURRET_MAXIMUM_POWER, TURRET_MAXIMUM_POWER);
        motor.setPower(pidOutput);
        return;
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
                ((this.motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) && this.motor.isBusy());
        // Vision tuning or running to position
    }
}
