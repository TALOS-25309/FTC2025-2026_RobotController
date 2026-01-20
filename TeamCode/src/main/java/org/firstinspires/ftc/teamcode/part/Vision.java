package org.firstinspires.ftc.teamcode.part;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Vision {
    private final Limelight3A LL;
    private double x,y,z;
    private boolean detected;

    enum PIPELINE {
        OBELISK_DETECTION,
        RED_GOAL,
        BLUE_GOAL
    };
    int RED_PIPELINE = 4;
    int BLUE_PIPELINE = 5;
    int OBELISK_PIPELINE = 6;



    public Vision(HardwareMap hardwareMap){
        LL = hardwareMap.get(Limelight3A.class, "Limelight");
        LL.pipelineSwitch(4);
    }

    public void start(){
        LL.setPollRateHz(250);
        LL.start();
    }
    public void setPipeline(PIPELINE pipeline) throws Exception {
        switch (pipeline){
            case RED_GOAL: LL.pipelineSwitch(RED_PIPELINE);
            case BLUE_GOAL: LL.pipelineSwitch(BLUE_PIPELINE);
            case OBELISK_DETECTION: LL.pipelineSwitch(OBELISK_PIPELINE);
            default: throw new Exception("잘못된 pipeline 접근 오류");
        }
    }

    public List<FiducialResult> fiducials;
    public LLStatus status;
    public double time;

    public void update(){

        LLResult result = LL.getLatestResult();
        status = LL.getStatus();

        if (result != null && result.isValid()) {
            time = result.getTimestamp();

            fiducials = result.getFiducialResults();
            if (fiducials.isEmpty()){
                detected = false;
            }
            else {
                detected = true;
                FiducialResult fiduciary = fiducials.get(0);
                this.x = fiduciary.getTargetPoseCameraSpace().getPosition().x;
                this.y = fiduciary.getTargetPoseCameraSpace().getPosition().y;
                this.z = fiduciary.getTargetPoseCameraSpace().getPosition().z;

            }
        }
        else{
            detected = false;
            time = -1;
        }
    }

    public int getFiducialID(){
        return fiducials.get(0).getFiducialId();
    }

    public LLStatus getStatus(){
        return status;
    }

    public boolean tagDetected(){
        return detected;
    }

    public double[] getPos(){
        return new double[]{x,y,z};
    }





    public void stop(){
        LL.stop();
    }


}
