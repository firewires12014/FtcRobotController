package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "blueFarCycle", group = "Robot")
public class blueFarCycle extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueFar VisionBlueFar = new VisionBlueFar(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamBlue"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        Pose2d startingPose = new Pose2d(-39.5,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        lift.lowerLift();
        outtake.lockPrimary();
        // Trajectory Declaration

        // right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .addTemporalMarker(()->{outtake.lockPrimary();})
                .lineToLinearHeading(new Pose2d(-50, 38, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{intake.up();lift.liftToHeight(60); outtake.autoDrop();})
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-51.5, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.specialStack();intake.inSlow();lift.liftToHeight(40);lift.holdLift(); outtake.intakePosition();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-45, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                //drive a little forward to stack
                .lineToLinearHeading(new Pose2d(-47, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {intake.in();})
                .waitSeconds(0.5)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.35)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(.25)
                //drive to back board
                .lineToLinearHeading(new Pose2d(30, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.75,()->{lift.liftToHeight(245); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(5);})
                .lineToLinearHeading(new Pose2d(55, 29.75, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.15,()->{outtake.intakePosition(); lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-38, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50.5, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{intake.specialStack(); lift.liftToHeight(40); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.25)
                //backs up
                .lineToLinearHeading(new Pose2d(-47.5, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()-> {intake.in();})
                .waitSeconds(1.2)
                //drive across field
                .lineToLinearHeading(new Pose2d(30, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); intake.specialStack();})
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {lift.liftToHeight(320); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{outtake.diffyPosition(4);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(43,10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(56,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .lineToLinearHeading(new Pose2d(55,35, Math.toRadians(180)),
//                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(53,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-42, 25, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{intake.up();lift.liftToHeight(60); outtake.autoDrop();})
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-51, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.specialStack();intake.inSlow();lift.liftToHeight(40);lift.holdLift(); outtake.intakePosition();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-45, 9, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                //drive a little forward to stack
                .lineToLinearHeading(new Pose2d(-48, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {intake.in();})
                .waitSeconds(0.5)
                //drive to back board
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.35)
                .lineToLinearHeading(new Pose2d(55, 12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.lockPixels(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{lift.liftToHeight(240); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.1,()->{outtake.diffyPosition(1);})
                .lineToLinearHeading(new Pose2d(50,25, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(225); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(1);})
                .lineToLinearHeading(new Pose2d(55, 33, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(30, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50.5, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{intake.specialStack(); lift.liftToHeight(60); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.25)
                //backs up
                .lineToLinearHeading(new Pose2d(-40, 10.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-42, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()-> {intake.in();})
                .waitSeconds(.5)
//                //drive across field
                .lineToLinearHeading(new Pose2d(45, 7, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); intake.specialStack();})
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{outtake.diffyPosition(1);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(47,7, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(56,31, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(49,31, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-29, 31, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.up();lift.liftToHeight(55); lift.holdLift();outtake.autoDrop();})
                .addTemporalMarker(()-> { outtake.releaseMain();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(-51.25, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()-> {intake.specialStack();intake.inSlow();lift.liftToHeight(40);lift.holdLift(); outtake.intakePosition();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-45, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                //drive a little forward to stack
                .lineToLinearHeading(new Pose2d(-47, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {intake.in();})
                .waitSeconds(0.5)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.25)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(.25)
                //drive to back board
                .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(59, 41.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{outtake.diffyPosition(1);})
                .lineToLinearHeading(new Pose2d(63,36.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.2)
                //yellow drop done
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-50.75, 11.75, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{intake.specialStack(); lift.liftToHeight(40); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.25)
                //backs up
                .lineToLinearHeading(new Pose2d(-46, 11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                .UNSTABLE_addTemporalMarkerOffset(-1.2, ()-> {intake.in();})
                .waitSeconds(1.2)
                //drive across field
                .lineToLinearHeading(new Pose2d(49, 10, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.lockPixels(); intake.specialStack();})
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.2,()->{outtake.diffyPosition(5);})

                .lineToLinearHeading(new Pose2d(64,33, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
            //    .waitSeconds(0.25)
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(61,35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

            VisionBlueFar.Location location = VisionBlueFar.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            intake.score();
            drive.followTrajectorySequence(rightMovementOne);
            sleep(3000000);
//            VisionBlueFar.Location location = VisionBlueFar.getLocation();
//            telemetry.addData("Location: ", location);
//
//            switch (location) {
//                case NOT_FOUND:
//                intake.score();
//                    drive.followTrajectorySequence(leftMovementOne);
//                    sleep(3000000);
//                    break;
//                case MIDDLE:
//                    intake.score();
//                    drive.followTrajectorySequence(middleMovementOne);
//                    sleep(300000);
//                    break;
//                case RIGHT:
//                    intake.score();
//                    drive.followTrajectorySequence(rightMovementOne);
//                    sleep(3000000);
//                    break;
//            }


            sleep(30000);
        }

    }

}

