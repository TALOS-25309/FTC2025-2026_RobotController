package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.feature.vision.Vision;
import org.firstinspires.ftc.teamcode.part.Turret;
import org.firstinspires.ftc.teamcode.part.shooter.Shooter;

@Config("ShooterFastSequence")
@TeleOp(name = "ShooterFastSequencex", group = "Tuning")
public class ShooterFastTest extends LinearOpMode {

    public static boolean TRIGGER_SHOOT = false; // Dashboard에서 체크 시 발사 시작

    // 각 공이 발사되는 간격 (ms) - 로봇의 물리적 발사 속도에 맞춰 튜닝 필요
    // 보통 인테이크/슈터 구조상 공 사이 간격은 매우 짧습니다 (예: 150~300ms)
    public static long BALL_INTERVAL_MS_1 = 200;
    public static long BALL_INTERVAL_MS_2 = 200;
    public static long BALL_INTERVAL_MS_3 = 200;

    public static double ANGLE_1 = 0.45;
    public static double ANGLE_2 = 0.48;
    public static double ANGLE_3 = 0.51;

    public static double VELOCITY = 6.0;

    private Shooter shooter;
    private Turret turret;
    private Vision vision;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter();
        turret = new Turret(new Vision()); // 간소화

        shooter.init(hardwareMap, telemetry);
        turret.init(hardwareMap, telemetry);

        waitForStart();
        shooter.start();

        while (opModeIsActive()) {
            // 1. 슈터 모터는 미리 돌려놓음 (준비 상태)
            shooter.shooterMotorUpper.setVelocity(VELOCITY, AngleUnit.RADIANS);
            shooter.shooterMotorLower.setVelocity(VELOCITY, AngleUnit.RADIANS);

            // 2. 트리거 감지
            if (TRIGGER_SHOOT) {
                runFastSequence();
                TRIGGER_SHOOT = false; // 실행 후 플래그 초기화
            }

            shooter.update();
            turret.update();
            telemetry.update();
        }
    }

    private void runFastSequence() {
        // [시작] 스토퍼 오픈 (3발이 연달아 지나감)
        shooter.cmdStopperOpen();

        // 1번 공 발사 시점 각도
        shooter.setAngle(ANGLE_1);
        sleep(BALL_INTERVAL_MS_1); // 다음 공이 올 때까지 아주 짧게 대기

        // 2번 공 발사 시점 각도
        shooter.setAngle(ANGLE_2);
        sleep(BALL_INTERVAL_MS_2);

        // 3번 공 발사 시점 각도
        shooter.setAngle(ANGLE_3);
        sleep(BALL_INTERVAL_MS_3);

        // [종료] 스토퍼 닫기
        shooter.cmdStopperClose();
    }
}