package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedClose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CRIredClose", group = "Robot")

public class redClose extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedClose VisionRedClose = new VisionRedClose(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamRed"), cameraMonitorViewId);
        camera.setPipeline(VisionRedClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(180));
        intake.allUp();
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(38, -37, Math.toRadians(140)))
                .addTemporalMarker(()->{intake.allUp(); intake.outSlow();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at backboard
                .lineToLinearHeading(new Pose2d(59, -42, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                //drop yellow
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                //reset claw/arm
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(230))
                //park
                .splineToConstantHeading(new Vector2d(63, -60), Math.toRadians(180))
                .build();

        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(34, -22, Math.toRadians(180)))
                .addTemporalMarker(()->{intake.allUp(); intake.out();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(59, -35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                .waitSeconds(.05)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(230))
                .splineToConstantHeading(new Vector2d(63, -60), Math.toRadians(180))
                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(15, -36, Math.toRadians(140)))
                .addTemporalMarker(()->{intake.allUp(); intake.outSlow();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(59, -29, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {
                    lift.liftToHeight(80); lift.holdLift();outtake.diffyPosition(3);
                telemetry.addData("liftPosition", lift.leftLift.getCurrentPosition());
                telemetry.update();})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.05)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.1)
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(245))
                .splineToConstantHeading(new Vector2d(63, -60), Math.toRadians(180))
                .build();


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });
        while (!opModeIsActive()) {

            VisionRedClose.Location location = VisionRedClose.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        waitForStart();
        outtake.intakePosition();
        drive.followTrajectorySequenceAsync(rightMovementOne);

        while (opModeIsActive()) {
            drive.update();



            VisionRedClose.Location location = VisionRedClose.getLocation();

//            switch (location) {
//                case LEFT:
//                    drive.followTrajectorySequence(leftMovementOne);
//                    stop();
////                    sleep(300000);
//                    break;
//
//                case MIDDLE:
//                    drive.followTrajectorySequence(middleMovementOne);
//                    stop();
////                    sleep(30000);
//                    break;
//
//                case RIGHT:
//                    drive.followTrajectorySequence(rightMovementOne);
//                    stop();
////                    sleep(300000);
//                    break;
       //     }


            //sleep(30000);
        }

    }

}

