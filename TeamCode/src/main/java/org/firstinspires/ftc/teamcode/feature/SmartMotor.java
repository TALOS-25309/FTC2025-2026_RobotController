package org.firstinspires.ftc.teamcode.feature;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.global.Global;

import java.util.Vector;

/**
 * Advanced DC Motor class.
 * <p>
 * This class provides an advanced interface for controlling DC motors with features: <br>
 * - PID control for precise positioning. <br>
 * - Synchronization with other motors. <br>
 * - Ability to set target positions and maximum power.
 */
public class SmartMotor {
    private final DcMotor motor;
    private final DcMotor encoder;
    private final String name;
    private static final Vector<SmartMotor> smartMotors = new Vector<>();

    private double encoderSign = 1.0;

    private double targetPosition = 0.0;
    private double previousTargetPosition = 0.0;
    private double motorMaximumPower = 1.0;

    public final PID pidController = new PID(0.1, 0, 0);

    private boolean isSynchronized = false;
    private final Vector<SmartMotor> synchronizedMotors = new Vector<>();

    private boolean isPIDActivated = false;

    public static void init() {
        smartMotors.clear();
    }

    /**
     * Constructor for SmartMotor. Uses the motor's built-in encoder.
     * @param motor The DcMotor instance to control.
     * @param name The name of the motor for identification.
     */
    public SmartMotor(@NonNull DcMotor motor, String name) {
        this.motor = motor;
        this.encoder = motor;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.setMotorDirection(DcMotor.Direction.FORWARD);
        this.setEncoderDirection(DcMotor.Direction.FORWARD);
        this.resetEncoder();
        this.name = name;
        smartMotors.add(this);
    }

    /**
     * Constructor for SmartMotor with a separate encoder.
     * @param motor The DcMotor instance to control.
     * @param encoder The DcMotor instance used as an encoder.
     * @param name The name of the motor for identification.
     */
    public SmartMotor(@NonNull DcMotor motor, DcMotor encoder, String name) {
        this.motor = motor;
        this.encoder = encoder;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.setMotorDirection(DcMotor.Direction.FORWARD);
        this.setEncoderDirection(DcMotor.Direction.FORWARD);
        this.resetEncoder();
        this.name = name;
        smartMotors.add(this);
    }

    /**
     * Sets the zero power behavior of the motor.
     * @param behavior The zero power behavior to set.
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        this.motor.setZeroPowerBehavior(behavior);
    }

    /**
     * Sets the direction of the motor.
     * @param direction The direction to set for the motor.
     */
    public void setMotorDirection(DcMotor.Direction direction) {
        this.motor.setDirection(direction);
    }

    /**
     * Sets the direction of the encoder.
     * @param direction The direction to set for the encoder.
     */
    public void setEncoderDirection(DcMotor.Direction direction) {
        if (direction == DcMotorSimple.Direction.FORWARD) {
            this.encoderSign = 1.0;
        } else {
            this.encoderSign = -1.0;
        }
    }

    /**
     * Resets the encoder of the motor.
     * <p>
     * This method stops the encoder and resets its position to zero.
     */
    public void resetEncoder() {
        if (Global.ENCODER_RESET) {
            this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /**
     * Sets the PID coefficients for the motor.
     * <p>
     * This method updates the PID controller with new coefficients.
     * @param p Proportional coefficient.
     * @param i Integral coefficient.
     * @param d Derivative coefficient.
     */
    public void setPID(double p, double i, double d) {
        this.pidController.updatePID(p, i, d);
    }

    /**
     * Sets the maximum power for the motor.
     * <p>
     * This method limits the maximum power that can be applied to the motor.
     * Especially, it is very important for PID control to prevent overshooting.
     * @param power The maximum power value (between 0.0 and 1.0).
     */
    public void setMotorMaximumPower(double power) {
        this.motorMaximumPower = power;
    }

    /**
     * Synchronizes this motor with another motor.
     * <p>
     * After synchronization, this motor will only controlled by the parent motor. <br>
     * Note: You cannot set position or power while synchronized.
     * @param motorName The name of the motor to synchronize with.
     */
    public void synchronizeWith(String motorName) {
        this.isSynchronized = true;
        SmartMotor parentMotor = getMotorByName(motorName);
        if (parentMotor == null) {
            throw new IllegalArgumentException("Motor with name " + motorName + " not found.");
        }
        parentMotor.synchronizedMotors.add(this);
    }

    /**
     * Sets the target position for the motor.
     * @param position The target position in encoder ticks.
     */
    public void setPosition(double position) {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot set position while synchronized.");
        }
        this.targetPosition = position;
        if (this.previousTargetPosition != this.targetPosition) {
            this.pidController.resetIntegral();
            this.previousTargetPosition = targetPosition;
        }
        this.activatePID();
    }

    /**
     * Gets the current position of the motor.
     * @return The current position in encoder ticks.
     */
    public double getCurrentPosition() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot get current position while synchronized.");
        }
        return this.encoder.getCurrentPosition() * this.encoderSign;
    }

    /**
     * Gets the target position of the motor.
     * @return The target position in encoder ticks.
     */
    public double getTargetPosition() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot get target position while synchronized.");
        }
        return this.targetPosition;
    }

    /**
     * Activates the PID control for the motor.
     * <p>
     * After activation, the motor will use PID control to reach the target position.
     */
    private void activatePID() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot activate PID while synchronized.");
        }
        this.isPIDActivated = true;
    }

    /**
     * Locks the current position as the target position for the motor.
     * <p>
     * PID control is very high cost and it is not recommended to use it when the motor is not moving.
     * Therefore, this method is used to lock the current position as the target position.
     * Moreover, it will turn off the PID, so you have to run it after the motion of motor is finished.
     */
    public void lockAsCurrentPower() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot lock current power while synchronized.");
        }
        this.targetPosition = this.encoder.getCurrentPosition() * this.encoderSign;
        this.previousTargetPosition = this.targetPosition;
        this.pidController.resetIntegral();
        this.isPIDActivated = false;
    }

    /**
     * Sets the power of the motor.
     * <p>
     * It will deactivate the PID control and set the motor to run at the specified power.
     * @param power The power value (between -1.0 and 1.0).
     */
    public void setPower(double power) {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot set power while synchronized.");
        }
        this.motor.setPower(power);
        for (SmartMotor synchronizedMotor : synchronizedMotors) {
            synchronizedMotor.motor.setPower(power);
        }
        this.targetPosition = this.encoder.getCurrentPosition() * this.encoderSign;
        this.previousTargetPosition = this.targetPosition;
        this.pidController.resetIntegral();
        this.isPIDActivated = false;
    }

    /**
     * Stops the motor and resets the target position.
     * <p>
     * This method stops the motor and resets the target position to the current position.
     * It also resets the PID controller's integral term and turn off the PID control.
     */
    public void stop() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot stop while synchronized.");
        }
        this.motor.setPower(0);
        for (SmartMotor synchronizedMotor : synchronizedMotors) {
            synchronizedMotor.motor.setPower(0);
        }
        this.targetPosition = this.motor.getCurrentPosition();
        this.previousTargetPosition = this.targetPosition;
        this.pidController.resetIntegral();
        this.isPIDActivated = false;
    }

    /**
     * Gets the motor instance.
     * @return The DcMotor instance controlled by this SmartMotor.
     */
    public DcMotor motor() {
        return motor;
    }

    /**
     * Update function for PID control.
     * <p>
     * This method should be called in your OpMode's loop to update the motor's position. <br>
     * To avoid overlapping update function calls,
     * it is recommended to use the {@link SmartMotor#updateAll()} method instead.
     */
    public void update() {
        if (isSynchronized) {
            return;
        }
        if (isPIDActivated) {
            double currentPosition = this.encoder.getCurrentPosition() * this.encoderSign;
            double error = targetPosition - currentPosition;
            double pidOutput = this.pidController.update(error, -motorMaximumPower, motorMaximumPower);
            this.motor.setPower(pidOutput);
            for (SmartMotor synchronizedMotor : synchronizedMotors) {
                synchronizedMotor.motor.setPower(pidOutput);
            }
        }
    }

    /**
     * Gets a SmartMotor by its name.
     * @param name The name of the motor to search for.
     * @return The SmartMotor instance with the specified name, or null if not found.
     */
    public static SmartMotor getMotorByName(String name) {
        for (SmartMotor smartMotor : smartMotors) {
            if (smartMotor.name.equals(name)) {
                return smartMotor;
            }
        }
        return null; // or throw an exception if preferred
    }

    /**
     * Updates all registered SmartMotors.
     * <p>
     * This method should be called in your OpMode's loop to update the state of all motors.
     */
    public static void updateAll() {
        for (SmartMotor smartMotor : smartMotors) {
            smartMotor.update();
        }
    }

    public static void emergencyStop() {
        for (SmartMotor smartMotor : smartMotors) {
            if (!smartMotor.isSynchronized) {
                smartMotor.setPower(0);
            }
        }
    }
}
