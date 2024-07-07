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

@Autonomous(name = "CRImiddle", group = "Robot")

public class CRImiddle extends LinearOpMode {
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
        Pose2d startingPose = new Pose2d(-12,-63.6, Math.toRadians(180));
        intake.allUp();
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(3, -29, Math.toRadians(180)))

                .build();

        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(3, -29, Math.toRadians(180)))

                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(-17, -38, Math.toRadians(150)))
                .addTemporalMarker(()->{intake.allUp(); intake.outRollerSlow();})
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ intake.die();})
                //add wait for partner if needed
                .setReversed(true)
                //spline away from purple
                .splineTo(new Vector2d(30, -50), Math.toRadians(0))
                //at backboard
                .splineToConstantHeading(new Vector2d(76, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {
                    lift.liftToHeight(180); lift.holdLift();outtake.diffyPosition(3);
                    telemetry.addData("liftPosition", lift.leftLift.getCurrentPosition());
                    telemetry.update();})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                .waitSeconds(.1)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(.25, ()-> {intake.die();})
                .setReversed(false)
                //away from backboard
                .splineToConstantHeading(new Vector2d(32, -60), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(3); outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-.8, ()-> {lift.lowerLift();})
                //through trusses to stack
                .lineToLinearHeading(new Pose2d(-49, -60, Math.toRadians(180)))
                //at stack
                .splineToConstantHeading(new Vector2d(-85.5, -34), Math.toRadians(90))
                .addTemporalMarker(()-> {lift.liftToHeight(50); lift.holdLift(); intake.in();})
                .waitSeconds(.3)
                .addTemporalMarker(()-> {intake.grabOne();})
                .waitSeconds(.75)
                .addTemporalMarker(()-> {intake.grabTwo();})
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-90))
                //away from stack
                .splineToConstantHeading(new Vector2d(-49, -60), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-.6, ()-> {intake.die(); })
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {outtake.lockPixels();})
                //through truss to backboard
                .lineToLinearHeading(new Pose2d(22, -60, Math.toRadians(180)))
                //at stack
                .splineToConstantHeading(new Vector2d(69, -40), Math.toRadians(0))
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
        drive.followTrajectorySequenceAsync(leftMovementOne);

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
