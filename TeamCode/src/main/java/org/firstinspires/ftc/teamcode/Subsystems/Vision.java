package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Vision.redPipeLine;
import org.firstinspires.ftc.teamcode.Vision.bluePipeLine;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
/*
public class Vision implements{
    public OpenCvCamera camera;
    private redPipeLine red = new redPipeLine();
    private bluePipeLine blue = new bluePipeLine();

    public Vision(HardwareMap hardwareMap){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMotionViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam 1"), cameraMonitorViewId);
    }

    public void enable() {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

            }

            @Override
            public void onError(int errorCode) {

            }
        }); {
            @Override
                    public void onOpened() {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                switch (){}
            }

    }
    }
}


 */