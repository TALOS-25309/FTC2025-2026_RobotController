package org.firstinspires.ftc.teamcode.part;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
//import static org.firstinspires.ftc.teamcode.part.Constants.DRIVE_POWER;

public class NewDrive implements Part{

    MecanumDrive mecanumDrive;
    Pose2d initPose = new Pose2d(0,0,0);


    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(hardwareMap, initPose);
    }

    @Override
    public void start() {
    }

    @Override
    public void update() {
        mecanumDrive.updatePoseEstimate();
    }

    public void setDrivePowers(double x, double y, double rx) {
        mecanumDrive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(y, x), rx
        ));
    }

    @Override
    public void stop() {
        setDrivePowers(0,0,0);
    }
}
