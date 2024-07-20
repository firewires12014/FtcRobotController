package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueClose;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CRIblueClose", group = "Robot")
public class CRIblueClose extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueClose VisionBlueClose = new VisionBlueClose(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBlue"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(28,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(43, 40, Math.toRadians(240)))
                .addTemporalMarker(()->{intake.allUp(); intake.outSlow();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at backboard
                .lineToLinearHeading(new Pose2d(65, 44, Math.toRadians(180)))
                //lift and diffy
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(1);})
                //drop yellow
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                //reset claw/arm
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(230))
                //.setReversed(true)
                //park
                .splineToConstantHeading(new Vector2d(63, 60), Math.toRadians(180))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(37, 22, Math.toRadians(180)))
                .addTemporalMarker(()->{intake.allUp(); intake.outSlow();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(65, 35, Math.toRadians(180)))
                //lift and diffy
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                .waitSeconds(.05)
                //yellow drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                //reset arm/claw
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(230))
                //park
                .splineToConstantHeading(new Vector2d(63, 60), Math.toRadians(180))
                .build();
        //Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(21, 40, Math.toRadians(240)))
                .addTemporalMarker(()->{intake.allUp(); intake.out();})
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(65, 29, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(180); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                .waitSeconds(.05)
                //first drop
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.1)
                .addTemporalMarker(()->{outtake.diffyPosition(3); outtake.intakePosition(); lift.lowerLift();})
                .setTangent(Math.toRadians(200))
                .splineToConstantHeading(new Vector2d(63, 60), Math.toRadians(180))
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

            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            intake.auto();
            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location:", location);
            telemetry.update();

            switch (location) {
                case NOT_FOUND:
                    drive.followTrajectorySequence(rightMovementOne);
                    stop();

                    break;
                case MIDDLE:
                    telemetry.addData("Middle:", "Activated");
                    telemetry.update();
                    drive.followTrajectorySequence(middleMovementOne);
                    stop();
                    break;
                case LEFT: //left
                    drive.followTrajectorySequence(leftMovementOne);
                    stop();
                    break;
            }
            sleep(30000);
        }

    }

}


