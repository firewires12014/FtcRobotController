package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

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

@Autonomous(name = "blueCloseCycleNew", group = "Robot")

public class blueCloseCycleNew extends LinearOpMode {
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
        outtake.intakePosition();
        lift.lowerLift();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(16,63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToLinearHeading(new Pose2d(45, 30, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.1, ()-> {lift.liftToHeight(175); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.25, ()-> {intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(60, 41, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(1); intake.allUp();})
                //drop 1
                .addTemporalMarker(()->{outtake.releasePixels();intake.up();})
                .waitSeconds(.15)
                //to stack
                .splineToConstantHeading(new Vector2d(16,56), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                .splineToConstantHeading(new Vector2d(-45,56), Math.toRadians(180))
                //at stack
                .lineToLinearHeading(new Pose2d(-53,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //grab one
                .UNSTABLE_addTemporalMarkerOffset(-.55, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.15)
                .addTemporalMarker(()->{intake.grabTwoSpecial();})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)))
                .splineToConstantHeading(new Vector2d(-45,59.2), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(43,57, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(42.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                //At backboard
                .lineToLinearHeading(new Pose2d(61,35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.45, ()-> {lift.liftToHeight(275); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.9, ()-> {intake.grabOne();outtake.diffyPosition(1); intake.out();})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();intake.die();})
                .waitSeconds(0.15)
                //to stack
                .splineToConstantHeading(new Vector2d(25,57), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .splineToConstantHeading(new Vector2d(-43, 57), Math.toRadians(180))
                //drive to stack/ at stack
                .lineToLinearHeading(new Pose2d(-51,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
               //back away from stack
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,57), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(43,57, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(42.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                //at backboard
                .lineToLinearHeading(new Pose2d(61,35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.45, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5);})
                //third drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(57,37, Math.toRadians(180)))
                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //spot to drop purple pixel
                .lineToLinearHeading(new Pose2d(37.5, 22.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //drop purple pixel
                .UNSTABLE_addTemporalMarkerOffset(-.1, ()-> {lift.liftToHeight(130); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> {intake.die();})
                //at back board
                .lineToConstantHeading(new Vector2d(61.75, 34),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(1); intake.allUp();})
                //drop 1
                .addTemporalMarker(()->{outtake.releasePixels();intake.up();})
                .waitSeconds(.15)
                //to stack
                .splineToConstantHeading(new Vector2d(16,58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                .splineToConstantHeading(new Vector2d(-40,59), Math.toRadians(180))
                //at stack
                .lineToLinearHeading(new Pose2d(-53,32, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake pixels
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.15)
                .addTemporalMarker(()->{intake.grabTwo();})
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,59.2), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back
                .lineToLinearHeading(new Pose2d(45,58, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                //At backboard
                .lineToLinearHeading(new Pose2d(61.5,37, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()-> {lift.liftToHeight(250); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {intake.grabOne();outtake.diffyPosition(1);})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.15)
                //to stack
                .splineToConstantHeading(new Vector2d(25,60), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .splineToConstantHeading(new Vector2d(-40, 58), Math.toRadians(180))
                //at stack
                .lineToLinearHeading(new Pose2d(-51,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,58), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back through truss
                .lineToLinearHeading(new Pose2d(43,58, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(43, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                //at backboard
                .lineToLinearHeading(new Pose2d(60.5,35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.45, ()-> {lift.liftToHeight(325); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5);})
//              //third drop
                .UNSTABLE_addTemporalMarkerOffset(-0.05, ()-> {outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(57,37, Math.toRadians(180)))
                .build();
        //Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //spot to drop purple pixel
                .lineToLinearHeading(new Pose2d(18.5, 30, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //drop purple pixel
                .UNSTABLE_addTemporalMarkerOffset(-.1, ()-> {lift.liftToHeight(130); lift.holdLift();outtake.diffyPosition(3);intake.outSlow();})
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {intake.die();})
                //at back board
                .lineToLinearHeading(new Pose2d(63.5, 28, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5); intake.allUp();})
                //first pixel drop
                .addTemporalMarker(()->{outtake.releasePixels();intake.up();})
                .waitSeconds(.15)
                //drive away from backboard
                .splineToConstantHeading(new Vector2d(16,56), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.8, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.7, ()-> {lift.lowerLift(); })
                //drive through truss to stack
                .splineToConstantHeading(new Vector2d(-42,56), Math.toRadians(180))
                //at stack
                .lineToLinearHeading(new Pose2d(-52.5,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake pixels
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {intake.in(); intake.grabOne();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.15)
                .addTemporalMarker(()->{intake.grabTwoSpecial();})
                .waitSeconds(0.25)
                //drive away from stack
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,56), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.55, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back through truss to backboard
                .lineToLinearHeading(new Pose2d(45,52, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(42.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, ()-> {outtake.lockPixels();})
                //spot to release pixels
                .lineToLinearHeading(new Pose2d(61,38, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()-> {lift.liftToHeight(250); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1.25, ()-> {intake.grabOne();outtake.diffyPosition(1);})
                //second drop
                .addTemporalMarker(()->{ outtake.releasePixels();})
                .waitSeconds(0.15)
                //to stack
                .splineToConstantHeading(new Vector2d(25,60), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {outtake.diffyPosition(3); intake.up();})
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift(); intake.up();})
                .splineToConstantHeading(new Vector2d(-40, 55), Math.toRadians(180))
                //at stack
                .lineToLinearHeading(new Pose2d(-50,34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-.4, ()-> {intake.in(); intake.grabFour();lift.liftToHeight(50); lift.holdLift();}) //grab one
                .waitSeconds(0.2)
                //drive away from stack
                .lineToLinearHeading(new Pose2d(-50,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,56), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(-0.6, ()-> {intake.die();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5, ()-> {lift.lowerLift();})
                //drive back through truss to backboard
                .lineToLinearHeading(new Pose2d(43,50, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(42.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25, ()-> {outtake.lockPixels();})
                //spot to release pixels
                .lineToLinearHeading(new Pose2d(62,38, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1.45, ()-> {lift.liftToHeight(310); lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.diffyPosition(5);})
//               //third drop
                .UNSTABLE_addTemporalMarkerOffset(-0.08, ()-> {outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(57,38, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {


            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            telemetry.addData("Location:", location);
            telemetry.update();
            switch (location) {
                case NOT_FOUND:
                    outtake.lockPixels();
                    intake.allUp();
                    drive.followTrajectorySequence(rightMovementOne);
                    stop();

                    break;
                case MIDDLE:
                    outtake.lockPixels();
                    intake.allUp();
                    telemetry.addData("Middle:", "Activated");
                    telemetry.update();
                    drive.followTrajectorySequence(middleMovementOne);
                    stop();
                    break;
                case LEFT:
                    outtake.lockPixels();
                    intake.allUp();
                    drive.followTrajectorySequence(leftMovementOne);
                    stop();
//                    sleep(30000);
                    break;
            }
            sleep(30000);
        }

    }

}




