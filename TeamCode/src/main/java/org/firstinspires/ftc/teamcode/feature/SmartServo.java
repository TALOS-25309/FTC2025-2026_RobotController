package org.firstinspires.ftc.teamcode.feature;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Vector;

/**
 * A class for managing servos with advanced features like synchronization and smooth transitions.
 * <p>
 * This class allows you to control servo positions with smooth transitions and synchronize multiple servos.
 * It provides methods to set positions, synchronize with other servos, and update the servo states.
 */
public class SmartServo {
    private final Servo servo;
    private double position;

    private double targetPosition;
    private double previousPosition;
    private long startTime;
    private long duration;

    private final String name;
    private static final Vector<SmartServo> smartServos = new Vector<SmartServo>();

    private boolean isSynchronized = false;
    private double syncPositionOffset = 0.0; // Difference in position for synchronized servos
    private final Vector<SmartServo> synchronizedServos = new Vector<SmartServo>();


    public static void init() {
        smartServos.clear();
    }

    /**
     * Constructor for SmartServo.
     * @param servo The Servo object to control.
     * @param name The name of the servo for identification.
     */
    public SmartServo(Servo servo, String name) {
        this.servo = servo;
        this.position = 0.5;
        this.targetPosition = 0.5;
        this.previousPosition = 0.5;
        this.startTime = 0;
        this.duration = 0;

        this.name = name;
        smartServos.add(this);
    }

    /**
     * Set the direction of the servo.
     * @param direction The direction to set for the servo.
     */
    public void setDirection(Servo.Direction direction) {
        this.servo.setDirection(direction);
    }

    /**
     * Synchronizes this servo with another servo.
     * <p>
     * After synchronization, this servo will only controlled by the parent servo. <br>
     * Note: You cannot set position while synchronized.
     * @param name The name of the parent servo to synchronize with.
     * @param syncPositionOffset The offset in position for this servo relative to the parent servo.
     */
    public void synchronizeWith(String name, double syncPositionOffset) {
        SmartServo parent = getServoByName(name);
        if (parent != null) {
            if (parent == this) {
                throw new IllegalArgumentException("Cannot synchronize itself");
            }
            this.isSynchronized = true;
            parent.synchronizedServos.add(this);
            this.syncPositionOffset = syncPositionOffset;
        } else {
            throw new IllegalArgumentException("No SmartServo found with name: " + name);
        }
    }

    /**
     * Sets the position of the servo.
     * @param position The target position for the servo, between 0.0 and 1.0.
     */
    public void setPosition(double position) {
        if (position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Position must be between 0.0 and 1.0");
        }

        if (isSynchronized) {
            throw new IllegalStateException("Cannot set position of synchronized servo. Use the parent servo instead.");
        }

        for (SmartServo smartServo : synchronizedServos) {
            if (position + smartServo.syncPositionOffset > 1.0) {
                position = 1.0 - smartServo.syncPositionOffset;
            } else if (position + smartServo.syncPositionOffset < 0.0) {
                position = 0.0 - smartServo.syncPositionOffset;
            }
        }

        this.targetPosition = position;
        this.previousPosition = position;
        this.position = position;

        servo.setPosition(position);
        for (SmartServo smartServo : synchronizedServos) {
            smartServo.position = position + smartServo.syncPositionOffset;
            smartServo.servo.setPosition(smartServo.position);
        }
    }

    /**
     * Sets the position of the servo with a duration for smooth transition.
     * <p>
     * This method allows you to set a target position and the duration over
     * which the servo should move to that position.
     * @param position The target position for the servo, between 0.0 and 1.0.
     * @param duration The duration in seconds over which the servo should move to the target position.
     */
    public void setPosition(double position, double duration) {
        if (position < 0.0 || position > 1.0) {
            throw new IllegalArgumentException("Position must be between 0.0 and 1.0");
        }
        if (isSynchronized) {
            throw new IllegalStateException("Cannot set position of synchronized servo. Use the parent servo instead.");
        }
        this.targetPosition = position;
        this.previousPosition = this.position;
        this.startTime = System.nanoTime();
        this.duration = (long) (duration * 1e9);
    }

    /**
     * Gets the current position of the servo.
     * @return The current position of the servo, between 0.0 and 1.0.
     */
    public double getPosition() {
        return position;
    }

    /**
     * Gets the target position of the servo.
     * @return The target position of the servo, between 0.0 and 1.0.
     */
    public double getTargetPosition() {
        if (isSynchronized) {
            throw new IllegalStateException("Cannot get target position of synchronized servo. Use the parent servo instead.");
        }
        return targetPosition;
    }

    /**
     * Gets the servo instance.
     * @return The Servo instance controlled by this SmartServo.
     */
    public Servo servo() {
        return servo;
    }

    /**
     * Updates the servo position based on the elapsed time since the last update.
     * <p>
     * This method should be called in your OpMode's loop to update the servo's position. <br>
     * To avoid overlapping update function calls,
     * it is recommended to use the {@link SmartMotor#updateAll()} method instead.
     */
    public void update() {
        if (duration == 0) {
            return;
        }

        double progress = (System.nanoTime() - startTime) / (double) duration;

        if (progress < 0) progress = 0;
        else if (progress > 1) progress = 1;

        progress = 1 - (1-progress) * (1-progress) * (1-progress);
        if (progress >= 1) {
            position = targetPosition;
            duration = 0;
        } else {
            position = (1 - progress) * this.previousPosition + progress * targetPosition;
        }

        for (SmartServo smartServo : synchronizedServos) {
            if (position + smartServo.syncPositionOffset > 1.0) {
                position = 1.0 - smartServo.syncPositionOffset;
            } else if (position + smartServo.syncPositionOffset < 0.0) {
                position = 0.0 - smartServo.syncPositionOffset;
            }
        }

        servo.setPosition(position);
        for (SmartServo smartServo : synchronizedServos) {
            smartServo.position = position + smartServo.syncPositionOffset;
            smartServo.servo.setPosition(smartServo.position);
        }
    }

    /**
     * Gets a SmartServo by its name.
     * @param name The name of the servo to search for.
     * @return The SmartServo instance with the specified name, or null if not found.
     */
    public static SmartServo getServoByName(String name) {
        for (SmartServo smartServo : smartServos) {
            if (smartServo.name.equals(name)) {
                return smartServo;
            }
        }
        return null;
    }

    /**
     * Updates all SmartServos in the system.
     * <p>
     * This method should be called in your OpMode's loop to update all SmartServos.
     * It iterates through all registered SmartServos and calls their update method.
     */
    public static void updateAll() {
        for (SmartServo smartServo : smartServos) {
            smartServo.update();
        }
    }

    public static void emergencyStop() {
        for (SmartServo smartServo : smartServos) {
            smartServo.servo.getController().pwmDisable();
        }
    }

    public static void normalState() {
        for (SmartServo smartServo : smartServos) {
            smartServo.servo.getController().pwmEnable();
        }
    }
}
