package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionRedFar;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redFar", group = "Robot")
public class redFar extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionRedFar VisionRedFar = new VisionRedFar(telemetry); // camera stuff
    SampleMecanumDrive drive;
    Outtake outtake;
    Lift lift;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(VisionRedFar); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        outtake = new Outtake(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        lift = new Lift(hardwareMap);
        //intake.resetIntake();
        outtake.lockPixels();
        Pose2d startingPose = new Pose2d(-39.5,-63.6, Math.toRadians(270));
        drive.setPoseEstimate(startingPose);

        // Trajectory Declaration

        // LEFT
        TrajectorySequence rightMovementOne = drive.trajectorySequenceBuilder(startingPose)
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(-34,-43, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-32,-35, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //put purple drop here
                .lineToLinearHeading(new Pose2d(-35,-25, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(32,-10),Math.toRadians(00),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(45,-43),Math.toRadians(00),
        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(48,-43, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        //MIDDLE
//        TrajectorySequence middleMovementOne = drive.trajectorySequenceBuilder(startingPose)
//                .setReversed(true)
//                        .splineTo(new Vector2d(-34,30),Math.toRadians(0),
//        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                //put purple drop here
//                .splineTo(new Vector2d(-15,15),Math.toRadians(270),
//                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

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
        outtake.lockSecondary();
        outtake.releaseMain();
        waitForStart();

        while (opModeIsActive()) {


            VisionRedFar.Location location = VisionRedFar.getLocation();


            switch (location) {
                case LEFT:
//                    drive.followTrajectorySequence(leftMovementOne);
                    //lift.moveLift(-.90f);
                    //sleep(200);
                    //lift.moveLift(0f);
                    //outtake.pivotEnding();
                    //sleep(1500);
                    //outtake.releaseSecondary();
                    break;
//                case MIDDLE:
//                    drive.followTrajectorySequence(middleMovementOne);
//                    break;

                case RIGHT:
                    drive.followTrajectorySequence(rightMovementOne);
                    break;
            }


            sleep(30000);
        }

    }

}

