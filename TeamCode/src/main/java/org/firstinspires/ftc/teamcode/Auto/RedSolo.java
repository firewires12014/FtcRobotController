package org.firstinspires.ftc.teamcode.Auto;

import android.os.FileUriExposedException;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Mecanum;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedSolo", group = "Robot")
public class RedSolo extends LinearOpMode {
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
        Pose2d startingPose = new Pose2d(-39.5,-63.6, Math.toRadians(180));
        drive.setPoseEstimate(startingPose);

        lift.lowerLift();
        outtake.lockPrimary();
        // Trajectory Declaration

        // right
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-43, -35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-39, -35, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up(); lift.liftToHeight(60); outtake.autoDrop();})
                //drop purple pixel
                .addTemporalMarker(()->outtake.releaseMain())
                .waitSeconds(0.55)
                // go to stack
                .lineToLinearHeading(new Pose2d(-71.75,-12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(40);lift.holdLift();outtake.intakePosition();intake.inSlow();})
                .waitSeconds(.25)
                //getting first white pixel
                .addTemporalMarker(()-> {intake.specialStack();intake.inSlow();lift.liftToHeight(40);lift.holdLift();})
                .waitSeconds(0.5)
                //back up from stack a little
                .lineToLinearHeading(new Pose2d(-64, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(32, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.2)
                .addTemporalMarker(()->{intake.score();})
                //drive a little forward to stack
                .lineToLinearHeading(new Pose2d(-69, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.2)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.35)
                // lock pixels
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.25)
                //drive across field
                .lineToLinearHeading(new Pose2d(41, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // getting lift ready
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(215); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(5);})
                //driving to backboard
                .lineToLinearHeading(new Pose2d(52, -44, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.75 ,()->{outtake.diffyPosition(3); intake.specialStack();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{ lift.lowerLift();})
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-65, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-70.75, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //put lift in correct position and begins intaking
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.5)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.2)
                //backs up
                .lineToLinearHeading(new Pose2d(-62, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                //moves forward
                .lineToLinearHeading(new Pose2d(-68, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                //drive across field
                .lineToLinearHeading(new Pose2d(30, -14, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); })
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(250); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(5);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(51,-31.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release pixels
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(49,-30.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.intakePosition();})

                .build();
        //MIDDLE
        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-59, -24, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ intake.up();lift.liftToHeight(60); outtake.autoDrop();})
                .addTemporalMarker(()->{outtake.releaseMain();})
                .waitSeconds(0.55)
                .lineToLinearHeading(new Pose2d(-71.75,-13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(40);lift.holdLift();outtake.intakePosition();intake.inSlow();})
                .waitSeconds(.25)
                .addTemporalMarker(()-> {intake.specialStack();intake.inSlow();lift.liftToHeight(40);lift.holdLift();})
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-64, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                .lineToLinearHeading(new Pose2d(-69, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.3)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.35)
                // grab 1st white pixel
                .addTemporalMarker(()-> {outtake.lockPixels(); })
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(35, -13, Math.toRadians(180)), //drive over
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{lift.liftToHeight(200); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(5);})
                .lineToLinearHeading(new Pose2d(50, -34.75, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{outtake.intakePosition(); lift.lowerLift();})
                .lineToLinearHeading(new Pose2d(40, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(30, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.75 ,()->{outtake.diffyPosition(3); intake.specialStack();})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.intakePosition(); })
                .UNSTABLE_addTemporalMarkerOffset(-0.15,()->{ lift.lowerLift();})
                //drives back to to stack
                .lineToLinearHeading(new Pose2d(-65, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-70.75, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //put lift in correct position and begins intaking
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.6)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.2)
                //backs up
                .lineToLinearHeading(new Pose2d(-62, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                //moves forward
                .lineToLinearHeading(new Pose2d(-68, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                //drive across field
                .lineToLinearHeading(new Pose2d(30, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); })
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(6);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(51.6,-31, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release pixels
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.15)
                .lineToLinearHeading(new Pose2d(49,-28, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //left
        TrajectorySequence leftMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .lineToLinearHeading(new Pose2d(-65, -30, Math.toRadians(180)), //drive to purple drop
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(55); lift.holdLift();outtake.autoDrop(); intake.up();})
                .addTemporalMarker(()-> { outtake.releaseMain();}) // drop purple
                .waitSeconds(0.5)
                // grab top pixel
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {outtake.intakePosition(); lift.liftToHeight(5); lift.holdLift();outtake.releasePixels();})
                // drive forward
                .lineToLinearHeading(new Pose2d(-70.75, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // start intaking
                .UNSTABLE_addTemporalMarkerOffset(-0.75, ()-> {intake.specialStack();})
                .UNSTABLE_addTemporalMarkerOffset(-.6, ()-> {;intake.inSlow();})
                .waitSeconds(0.2) //put back to 0.4 if no work
                // backup
                .lineToLinearHeading(new Pose2d(-64, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // lift intake
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                // lower intake
                .addTemporalMarker(()->{intake.score();})
                // drive forward
                .lineToLinearHeading(new Pose2d(-68, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                // intakes fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.3)
                // stops intake and lowers lift
                .addTemporalMarker(()->{intake.die();lift.lowerLift();})
                .waitSeconds(.25)
                // grab 1st white pixel
                .addTemporalMarker(()-> {outtake.lockPixels(); })
//                .waitSeconds(0.25)
                // drive across field
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(0.25)
                //lock pixels
                .addTemporalMarker(()->{outtake.lockPixels();})
                //putting lift to release height
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {outtake.lockPixels();lift.liftToHeight(240); lift.holdLift(); outtake.diffyPosition(3);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(1);})
                //driving to spot to release pixels
                .lineToLinearHeading(new Pose2d(50,-32.5, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //release pixels
                .addTemporalMarker(()->{outtake.releasePixels();})
                .waitSeconds(0.25)
                //getting ready to drive across field
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //puts lift down
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(3); intake.specialStack();})
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{outtake.intakePosition(); lift.lowerLift();})
                //drives to stack
                .lineToLinearHeading(new Pose2d(-60, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-70.75, -11, Math.toRadians(180)))
                //put lift in correct position and begins intaking
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{ lift.liftToHeight(50); lift.holdLift();})
                .waitSeconds(0.40)
                .addTemporalMarker(()-> {intake.superStack();})
                .addTemporalMarker(()->{;intake.in();})
                .waitSeconds(.6)
                .addTemporalMarker(()-> {intake.stackHeightThree();})
                .waitSeconds(0.2)
                //backs up
                .lineToLinearHeading(new Pose2d(-62, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(()->{intake.up();})
                .waitSeconds(0.25)
                .addTemporalMarker(()->{intake.score();})
                //moves forward
                .lineToLinearHeading(new Pose2d(-68, -11, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //intake fast
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {intake.in();})
                .waitSeconds(0.4)
                //drive across field
                .lineToLinearHeading(new Pose2d(30, -13, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //stop intake and lower lift
                .UNSTABLE_addTemporalMarkerOffset(-2.5,()->{intake.die(); lift.lowerLift();})
                //lock pixels
                .UNSTABLE_addTemporalMarkerOffset(-2, ()-> {outtake.lockPixels(); })
                //lift to correct spot
                .UNSTABLE_addTemporalMarkerOffset(-1, ()-> {lift.liftToHeight(300); lift.holdLift(); outtake.diffyPosition(1);})
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{outtake.diffyPosition(1);})
                //moves to spot to release pixels
                .lineToLinearHeading(new Pose2d(50,-34, Math.toRadians(180)),
                        Mecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        Mecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-0.25,()->{outtake.releasePixels();})
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

        while (opModeIsActive()) {

            VisionRedFar.Location location = VisionRedFar.getLocation();
            telemetry.addData("Location: ", location);

            switch (location) {
                case LEFT:
                    intake.score();
                    sleep(300);
                    intake.specialStack();
                    drive.followTrajectorySequence(leftMovementOne);
                    sleep(3000000);
                    break;
                case MIDDLE:
                    intake.score();
                    drive.followTrajectorySequence(middleMovementOne);
                    sleep(300000);
                    break;
                case RIGHT:
                    intake.score();
                    drive.followTrajectorySequence(rightMovementOne);
                    sleep(3000000);
                    break;
            }


            sleep(30000);
        }

    }

}

