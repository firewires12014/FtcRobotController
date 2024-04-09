package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

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

@Autonomous(name = "redCloseCycle", group = "Robot")

public class redCloseCycle extends LinearOpMode {
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
        lift.lowerLift();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(16,-63.6, Math.toRadians(180));

        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(25, -32, Math.toRadians(180)),
                       Mecanum.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                       Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.stack();})
                .lineToLinearHeading(new Pose2d(54, -44, Math.toRadians(180)),
                Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-4, ()-> {lift.liftToHeight(220); lift.holdLift();})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(3); })
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5);})
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
                .lineToLinearHeading(new Pose2d(18, -25, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, ()-> {lift.liftToHeight(210); lift.holdLift();outtake.diffyPosition(3);intake.outRoller();})
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {intake.die();})
                .lineToLinearHeading(new Pose2d(51, -35.75, Math.toRadians(180)), //at back board
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {lift.liftToHeight(130); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()-> {lift.holdLift(); outtake.diffyPosition(5);})
                .addTemporalMarker(()->{outtake.releasePixels();}) //first drop
                .waitSeconds(0.15)
                //to stack
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> { intake.up(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition(); lift.liftToHeight(0);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .lineToLinearHeading(new Pose2d(-40,-58, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-50, -35), Math.toRadians(180),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-64,-35, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(75); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.grabTwo();})
                .waitSeconds(0.15)
                .splineToConstantHeading(new Vector2d(-40,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.45, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52,-37, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5);})
                .addTemporalMarker(()->{ outtake.releasePixels();}) //second drop
                .waitSeconds(0.15)

                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                 .lineToLinearHeading(new Pose2d(6.5, -32, Math.toRadians(180)), //purple drop
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.1, ()-> {lift.liftToHeight(210); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {intake.die();})
                .lineToLinearHeading(new Pose2d(50, -29, Math.toRadians(180)), //at back board
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(1); intake.allUp();})
                .addTemporalMarker(()->{outtake.releasePixels();intake.up();})
                .waitSeconds(.15)
                //drop 1
                //to stack
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); })
                .lineToLinearHeading(new Pose2d(-55,-58, Math.toRadians(180)))
        .lineToLinearHeading(new Pose2d(-64.5,-34.75, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(75); lift.holdLift();}) //grab one
                .waitSeconds(0.15)
                .addTemporalMarker(()->{intake.grabTwoSpecial();})
                .waitSeconds(0.3)
        .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)))
        .splineToConstantHeading(new Vector2d(-40,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(45,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52,-39, Math.toRadians(180)),
        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(200); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.9, ()-> {intake.grabOne();outtake.diffyPosition(5);})
                .addTemporalMarker(()->{ outtake.releasePixels();}) //second drop
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(16,-58.5), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .lineToLinearHeading(new Pose2d(-55,-58, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-64.5,-34.75, Math.toRadians(180)),  //drive to stack/ at stack
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(75); lift.holdLift();}) //grab one
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-60,-50, Math.toRadians(180)), //temp change
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-40,-58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40,-58, Math.toRadians(180))) //drive back
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                .lineToLinearHeading(new Pose2d(52,-50, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(150); lift.holdLift();outtake.diffyPosition(3);})
                .addTemporalMarker(()->{ outtake.releasePixels();}) //second drop


                .build();


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
//                camera.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
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
            intake.allUp();
            drive.followTrajectorySequence(leftMovementOne);




//            VisionRedClose.Location location = VisionRedClose.getLocation();
//
//            switch (location) {
//                case LEFT:
//                    intake.allUp();
//                    drive.followTrajectorySequence(leftMovementOne);
//                    sleep(300000);
//                    break;
//
//                case MIDDLE:
//                   intake.allUp();
//                    drive.followTrajectorySequence(middleMovementOne);
//                    sleep(30000);
//                    break;
//
//                case RIGHT:
//                  intake.allUp();
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


