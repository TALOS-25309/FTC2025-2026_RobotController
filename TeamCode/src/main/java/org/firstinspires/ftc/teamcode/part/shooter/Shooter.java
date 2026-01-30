package org.firstinspires.ftc.teamcode.part.shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.feature.Schedule;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;
import static org.firstinspires.ftc.teamcode.part.Constants.*;

import org.firstinspires.ftc.teamcode.part.Constants;
import org.firstinspires.ftc.teamcode.part.Part;


public class Shooter implements Part {
    public DcMotorEx shooterMotorUpper;
    public DcMotorEx shooterMotorLower;

    Servo shooterServo;
    Servo stopper;


    public ShooterState shooterState;
    private double hoodAngle;
    private boolean isBusy;


    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterMotorUpper = hardwareMap.get(DcMotorEx.class, "shooter_motor1");
        shooterMotorLower = hardwareMap.get(DcMotorEx.class, "shooter_motor2");

        shooterMotorUpper.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotorLower.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotorUpper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorLower.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterServo = hardwareMap.get(Servo.class, "servo_hood");
        stopper = hardwareMap.get(Servo.class, "servo_stopper");
    }

    @Override
    public void start() {
        shooterMotorUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotorLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        cmdShooterRun();

        shooterServo.setPosition(1);
        stopper.setPosition(SHOOTER_STOPPER_CLOSE_ANGLE);

        shooterState = ShooterState.STOP;

    }

    @Override
    public void update() {
        shooterMotorUpper.setVelocityPIDFCoefficients(0,0,0,SHOOTER_UPPER_PID_F);
        shooterMotorLower.setVelocityPIDFCoefficients(0,0,0,SHOOTER_LOWER_PID_F);

        double velTick = shooterMotorUpper.getVelocity(); // Tick/s
        double ticksPerRev = 145.1;
        double RPM = (velTick / ticksPerRev) * 60.0;
//        TelemetrySystem.addClassData("Shooter", "RPM", RPM);
//        TelemetrySystem.addClassData("Shooter", "velocity lower", shooterMotorLower.getVelocity(AngleUnit.RADIANS));
//        TelemetrySystem.addClassData("Shooter", "velocity upper", shooterMotorUpper.getVelocity(AngleUnit.RADIANS));

        TelemetrySystem.addClassData("Shooter", "Angle", shooterServo.getPosition());


        double appliedVelocity = AAA_SHOOTER_VELOCITY;
        double realVelocity = shooterMotorUpper.getVelocity(AngleUnit.RADIANS);
        TelemetrySystem.addClassData("Shooter","applied Velocity", appliedVelocity);
        TelemetrySystem.addClassData("Shooter","real Velocity", realVelocity);



    }

    @Override
    public void stop() {
        cmdShooterStop();
        shooterServo.getController().pwmDisable();
    }


    public double getInterpolatedAngle(double distance) {
        // 거리 배열 가져오기
        double[] dists = SHOOTER_DISTANCES;
        double[] angles = SHOOTER_ANGLES;

        // 1. 범위 밖 예외 처리 (가장 가까운 값 사용)
        if (distance <= dists[0]) return angles[0];
        if (distance >= dists[dists.length - 1]) return angles[angles.length - 1];

        // 2. 이분 탐색 등으로 구간 찾기 (데이터가 적으면 for문도 무관)
        int index = 0;
        for (int i = 0; i < dists.length - 1; i++) {
            if (distance >= dists[i] && distance <= dists[i+1]) {
                index = i;
                break;
            }
        }

        // 3. 선형 보간 (Linear Interpolation) 공식 적용
        // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
        double x1 = dists[index];
        double x2 = dists[index + 1];
        double y1 = angles[index];
        double y2 = angles[index + 1];

        double resultAngle = y1 + (distance - x1) * (y2 - y1) / (x2 - x1);
        return resultAngle;
    }

    public boolean isBusy(){
        return isBusy;
    }
    public void setBusy(boolean v){
        isBusy = v;
    }


    public void setAngle(double angle){
        shooterServo.setPosition(angle);
    }
    public double getAngle(){
        return shooterServo.getPosition();
    }


    // Commands
    public void cmdShooterRun(){
        // m/s / m = 1/s
        shooterMotorUpper.setVelocity(AAA_SHOOTER_VELOCITY, AngleUnit.RADIANS);
        shooterMotorLower.setVelocity(AAA_SHOOTER_VELOCITY, AngleUnit.RADIANS);

        shooterState = ShooterState.RUN;
    }
    public void cmdShooterStop(){
        shooterMotorUpper.setPower(0);
        shooterMotorLower.setPower(0);
        shooterState = ShooterState.STOP;
    }



    public void cmdSetAngleByDist(double distance){
        hoodAngle = getInterpolatedAngle(distance);
        shooterServo.setPosition(hoodAngle);
    }

    public void cmdShootThreeBalls(double distance){

        double angle1, angle2, angle3;

        angle1 = getInterpolatedAngle(distance);
        angle2 = angle1 + SHOOTER_ANGLE_DIFF;
        angle3 = angle1 + SHOOTER_ANGLE_DIFF;

        Schedule.addTask(
            ()->{
                shooterServo.setPosition(angle1);
            },
            Schedule.RUN_INSTANTLY
        );

        Schedule.addTask(
                ()->{
                    cmdStopperOpen();
                    shooterServo.setPosition(angle2);
                },
                SHOOTER_TIME_INTERVAL_ONE
        );

        Schedule.addTask(
                ()->{
                    shooterServo.setPosition(angle3);
                },
                SHOOTER_TIME_INTERVAL_TWO
        );

//        Schedule.addTask(
//
//        );





    }




    public void cmdStopperOpen(){
        stopper.setPosition(SHOOTER_STOPPER_OPEN_ANGLE);
    }
    public void cmdStopperClose(){
        stopper.setPosition(SHOOTER_STOPPER_CLOSE_ANGLE);
    }

 }
