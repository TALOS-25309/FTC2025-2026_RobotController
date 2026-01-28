package org.firstinspires.ftc.teamcode.feature.vision;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import static org.firstinspires.ftc.teamcode.part.Constants.*;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.feature.TelemetrySystem;

import java.util.List;

public class Vision {
    private  Limelight3A LL;
    private YawPitchRollAngles rotation; // 태그의 회전 정보
    private Position positionOnCamera;
    private double angle, distance;
    private boolean detected;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
    }
    public void start() {;
        LL.setPollRateHz(40);
        LL.start();
    }
    public void setPipeline(PIPELINE pipeline) {
        switch (pipeline){
            case RED_GOAL:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
            case BLUE_GOAL:
                LL.pipelineSwitch(BLUE_PIPELINE);
                break;
            case OBELISK_DETECTION:
                LL.pipelineSwitch(OBELISK_PIPELINE);
                break;
            default:
                LL.pipelineSwitch(RED_PIPELINE);
                break;
        }
    }

    public List<FiducialResult> fiducials;
    public LLStatus status;
    public double time;

    public void update(){

        LLResult result = LL.getLatestResult();
        status = LL.getStatus();
        TelemetrySystem.addClassData("VISION", "LL Temp", status.getTemp());
        TelemetrySystem.addClassData("VISION", "LL CPU", status.getCpu());


        if (result != null && result.isValid()) {
            time = result.getTimestamp();

            fiducials = result.getFiducialResults();
            TelemetrySystem.addClassData("VISION", "detected",detected);
            if (fiducials.isEmpty()){
                detected = false;
            }
            else {
                detected = true;
                FiducialResult fiduciary = fiducials.get(0);

//                positionOnCamera = fiduciary.getTargetPoseCameraSpace().getPosition();
//                rotation = fiduciary.getTargetPoseCameraSpace().getOrientation();
                angle = result.getTx();
                distance = fiduciary.getTargetPoseRobotSpace().getPosition().z;

            } // 17.73 cm
        }
        else{
            detected = false;
            time = -1;
        }
    }

    public int getFiducialID(){
        return fiducials.get(0).getFiducialId();
    }

    public double getTimestamp(){
        return LL.getLatestResult().getTimestamp();
    }

    public boolean tagDetected(){
        return detected;
    }

    public double getDistance(){
        return distance;
    }

    public double getAngle(){
        return angle;
    }


//    public Position getDistance(){
//        return
//    }



    public void stop(){
        LL.stop();
    }


}
