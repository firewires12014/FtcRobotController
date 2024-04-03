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

@Autonomous(name = "redClose", group = "Robot")

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
                .lineToLinearHeading(new Pose2d(25, -32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.1)
                .addTemporalMarker(()->{intake.allUp(); intake.out();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{ intake.die();})
                .addTemporalMarker(()->{lift.liftToHeight(220); lift.holdLift();})
                .addTemporalMarker(()->{outtake.diffyPosition(3); })
                .addTemporalMarker(()->{outtake.diffyPosition(5);})
                .lineToLinearHeading(new Pose2d(54, -44, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .waitSeconds(.2)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(51, -44, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();lift.liftToHeight(0); intake.score();})
                .lineToLinearHeading(new Pose2d(55, -57, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(20, -23, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.allUp(); intake.out();})
                .waitSeconds(0.05)
                .addTemporalMarker(()->{ intake.die();})
                .lineToLinearHeading(new Pose2d(51, -34, Math.toRadians(180)), //at back board
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                .waitSeconds(.05)
                .addTemporalMarker(()->{outtake.releasePixels();}) //first drop
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(48, -34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(55, -57, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(9, -32, Math.toRadians(180)), //purple drop
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {lift.liftToHeight(245); lift.holdLift();outtake.diffyPosition(3);})
                .addTemporalMarker(()->{intake.allUp(); intake.outRoller();})
                .waitSeconds(.12)
                .addTemporalMarker(()->{intake.die();})
                .lineToLinearHeading(new Pose2d(5.5, -32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(.25)
                .lineToLinearHeading(new Pose2d(50, -29, Math.toRadians(180)), //at back board
                        Mecanum.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(1);})
                .waitSeconds(.1)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(.5)
                .lineToLinearHeading(new Pose2d(46, -29, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(55, -57, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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

        while (opModeIsActive()) {
            drive.followTrajectorySequence(rightMovementOne);




//            VisionRedClose.Location location = VisionRedClose.getLocation();
//
//            switch (location) {
//                case LEFT:
//                    drive.followTrajectorySequence(leftMovementOne);
//                    sleep(300000);
//                    break;
//
//                case MIDDLE:
//                    drive.followTrajectorySequence(middleMovementOne);
//                    sleep(30000);
//                    break;
//
//                case RIGHT:
//                    drive.followTrajectorySequence(rightMovementOne);
//                    sleep(300000);
//                    break;
//            }
//
//
            sleep(30000);
        }

    }

}



