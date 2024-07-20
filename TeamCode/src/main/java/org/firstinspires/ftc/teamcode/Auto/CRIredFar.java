package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedFar;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "CRIredFar", group = "Robot")

public class CRIredFar extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedFar VisionRedFar = new VisionRedFar(telemetry); // camera stuff
    Mecanum drive;
    Outtake outtake;
    Lift lift;
    Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WebcamRed"), cameraMonitorViewId);
        camera.setPipeline(VisionRedFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new Mecanum(hardwareMap);
        lift = new Lift(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
        //intake.resetIntake();
        outtake.lockPrimary();
        Pose2d startingPose = new Pose2d(-61,-63.6, Math.toRadians(180));
        intake.allUp();
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // Right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(-55, -34), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})//drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(.15)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {outtake.intakePosition(); lift.liftToHeight(55);lift.holdLift();})
                //at stack
                .lineToLinearHeading(new Pose2d(-79, -10, Math.toRadians(180)))
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabOne(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.7, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.85,()-> {outtake.lockPixels();})
                .setReversed(true)
                //drive across field
                .lineToLinearHeading(new Pose2d(60, -11, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(270);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(0);})
                //spline to backboard
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(81.75, -43), Math.toRadians(0))
                .waitSeconds(.1)
                //first drop
                .addTemporalMarker(()-> {outtake.releasePixels();})
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
                .waitSeconds(.4)
                .setReversed(false)
                //move away from backboard
                .lineToConstantHeading(new Vector2d(60, -11))
                //at stack
                .lineToConstantHeading(new Vector2d(-79.5, -10))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.grabThree();})
                .waitSeconds(.8)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.7, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.85,()-> {outtake.lockPixels();})
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(40, -11))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(370);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                //spline to backboard
                .splineToConstantHeading(new Vector2d(81, -31), Math.toRadians(0))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {outtake.releasePixels();})
                //park
                .lineToLinearHeading(new Pose2d(76, -13, Math.toRadians(180)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
                .build();

        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple
                .lineToConstantHeading(new Vector2d(-72, -25))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})//drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(.15)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {outtake.intakePosition(); lift.liftToHeight(55);lift.holdLift();})
                //at stack
                .lineToLinearHeading(new Pose2d(-80, -10, Math.toRadians(180)))
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabOne(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.7, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.85,()-> {outtake.lockPixels();})
                .setReversed(true)
                //drive across field
                .lineToLinearHeading(new Pose2d(60, -11, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(270);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(0);})
                //spline to backboard
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(81.5, -36), Math.toRadians(0))
                .waitSeconds(.1)
                //first drop
                .addTemporalMarker(()-> {outtake.releasePixels();})
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
                .waitSeconds(.4)
                .setReversed(false)
                //move away from backboard
                .lineToConstantHeading(new Vector2d(60, -11))
                //at stack
                .lineToConstantHeading(new Vector2d(-79.5, -10))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.grabThree();})
                .waitSeconds(.8)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.7, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.85,()-> {outtake.lockPixels();})
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(40, -11))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(370);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                //spline to backboard
                .splineToConstantHeading(new Vector2d(81, -31), Math.toRadians(0))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {outtake.releasePixels();})
                //park
                .lineToLinearHeading(new Pose2d(76, -13, Math.toRadians(180)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
                .build();
        //Left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                //purple drop
                .lineToConstantHeading(new Vector2d(-78.5, -33))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})//drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(.15)
                //getting into position for first white pixel
                .addTemporalMarker(()-> {outtake.intakePosition(); lift.liftToHeight(55);lift.holdLift();})
                //at stack
                .lineToLinearHeading(new Pose2d(-79.5, -11, Math.toRadians(180)))
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabOne(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.7, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.85,()-> {outtake.lockPixels();})
                .setReversed(true)
                //drive across field
                .lineToLinearHeading(new Pose2d(40, -11, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(290);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(0);})
                //spline to backboard
                .splineToConstantHeading(new Vector2d(81.75, -28.5), Math.toRadians(0))
                .waitSeconds(.1)
                //first drop
                .addTemporalMarker(()-> {outtake.releasePixels();})
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
                .waitSeconds(.4)
                .setReversed(false)
                .lineToConstantHeading(new Vector2d(60, -11))
                //at stack
                .lineToConstantHeading(new Vector2d(-79.5, -10))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {lift.liftToHeight(40);lift.holdLift();})
                .addTemporalMarker(()-> {intake.grabTwo(); intake.in();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.grabThree();})
                .waitSeconds(.7)
                .addTemporalMarker(()-> {intake.up();})
                .setTangent(90)
                .UNSTABLE_addTemporalMarkerOffset(.6, ()-> {intake.die(); lift.lowerLift();})
                .UNSTABLE_addTemporalMarkerOffset(.75,()-> {outtake.lockPixels();})
                .setReversed(true)
                .lineToConstantHeading(new Vector2d(40, -11))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(370);lift.holdLift();outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-.5, ()-> {lift.holdLift();outtake.diffyPosition(5);})
                //spline to backboard
                .splineToConstantHeading(new Vector2d(81.75, -28.5), Math.toRadians(0))
                .waitSeconds(.2)
                .addTemporalMarker(()-> {outtake.releasePixels();})
                //park
                .lineToLinearHeading(new Pose2d(76, -13, Math.toRadians(180)))
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.3,()->{outtake.diffyPosition(3); outtake.intakePosition();})
                .UNSTABLE_addTemporalMarkerOffset(.4, ()-> { lift.lowerLift();})
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

            VisionRedFar.Location location = VisionRedFar.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        waitForStart();
        outtake.intakePosition();

        while (opModeIsActive()) {




            VisionRedFar.Location location = VisionRedFar.getLocation();

            switch (location) {
                case LEFT:
                    drive.followTrajectorySequence(leftMovementOne);
                    stop();
//                    sleep(300000);
                    break;

                case MIDDLE:
                    drive.followTrajectorySequence(middleMovementOne);
                    stop();
//                    sleep(30000);
                    break;

                case RIGHT:
                    drive.followTrajectorySequence(rightMovementOne);
                    stop();
//                    sleep(300000);
                    break;
            }


            sleep(30000);
        }

    }

}

