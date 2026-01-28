package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;

@Config("AutoShooterManager")
public class AutoShooterManager {
    private DcMotorEx shooterMotor1, shooterMotor2;

    private Servo stopper;
    private Servo hood;

    private AutoTurret turret;

    public static double shooterSpeed = 105;
    public static double stopperUp=0.77, stopperDown=0.63;
    public static double hoodUp=0, hoodDown = 1;
    public static double shootDelay = 2000;

    boolean visionAligning = false;
    boolean shooting = false;
    long shotAt;

    AutoShooterManager(
            DcMotorEx shooter1, DcMotorEx shooter2,
            Servo stopper,
            Servo hood,
            DcMotorEx turretMotor,
            Vision limelight
    ) {
        this.shooterMotor1 = shooter1; this.shooterMotor2 = shooter2;
        this.stopper = stopper;
        this.hood = hood;
        this.turret = new AutoTurret(
                turretMotor,
                limelight
        );
    }

    void init() {
        this.shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        this.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.turret.init();
    }

    void start() {
        this.turret.start();
        closeStopper();
        setHood(0);
    }

    void startMotor() {
        this.shooterMotor1.setVelocity(shooterSpeed, AngleUnit.RADIANS);
        this.shooterMotor2.setVelocity(shooterSpeed, AngleUnit.RADIANS);
    }
    void stopMotor() {
        this.shooterMotor1.setPower(0);
        this.shooterMotor2.setPower(0);
    }

    void openStopper() {
        this.stopper.setPosition(stopperUp);
    }
    void closeStopper() {
        this.stopper.setPosition(stopperDown);
    }

    void setHood(double x) {
        x = Math.max(0, Math.min(1, x));
        // 0 is down, 1 is up
        double position = x * hoodUp + (1-x) * hoodDown;
        this.hood.setPosition(position);
    }

    void lookAt(double turretRad, double hoodPosition) {
        this.turret.gaze(turretRad);
        setHood(hoodPosition);
    }

    void shoot() {
        shooting = true;
        closeStopper();
        startMotor();
        visionAligning = true;
        this.turret.startVisionAlign();
    }

    void updateShoot() {
        if (shooting && visionAligning && (!this.turret.visionActive)) { // Align ended
            visionAligning = false;
            openStopper();
            shotAt = System.currentTimeMillis();
        }
        if (shooting && (!visionAligning) && (System.currentTimeMillis() - shotAt > shootDelay)) {
            closeStopper();
            shooting = false;
        }
    }

    void update() {
        updateShoot();
        this.turret.update();
    }

    public boolean isShooting() {
        return this.shooting;
    }
}
