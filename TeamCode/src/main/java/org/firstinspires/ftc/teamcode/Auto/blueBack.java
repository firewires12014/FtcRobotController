package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.VisionBlueClose;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "blueBack", group = "Robot")
@Disabled
public class blueBack extends LinearOpMode {
    public OpenCvCamera camera;
    private VisionBlueClose VisionBlueClose = new VisionBlueClose(telemetry); // camera stuff
    DriveTrain driveTrain;
    Intake intake;
    Outtake outtake;
    Lift lift;

    static final double COUNTS_PER_MOTOR_REV = 384.5; // <----- this is probably wrong. check it.
    static final double DRIVE_GEAR_REDUCTION = 1.0; // <----- fiddle with this. test until the robot goes 12 inches with encDrive
    static final double WHEEL_DIAMETER_INCHES = 3.78; // <----- this is right
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(VisionBlueClose); // this is what gets the camera going.
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        lift = new Lift(hardwareMap);

        //intake.resetIntake();
        outtake.lockPixels();


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
        outtake.lockSecondary();
        outtake.releaseMain();
        waitForStart();

        while (opModeIsActive()) {
            VisionBlueClose.Location location = VisionBlueClose.getLocation();
            switch (location) {
                case NOT_FOUND:
                    encoderDrive(23, -.4);
                    sleep(1000);
                    turn(85, .3);
                    sleep(1000);
                    encoderDrive(10, .4);
                    sleep(500);
                    encoderDrive(8, -.4);
                    sleep(750);
                    outtakePixel();
                    sleep(500);
                    encoderDrive(1.5,0.4);
                    sleep(500);
                    strafeDrive(6,0.4);
                    encoderDrive(37.5, -.4);
                    sleep(1000);
                    lift.moveLift(-.75f);
                    sleep(200);
                    lift.moveLift(0f);
                    outtake.pivotEnding();
                    sleep(1500);
                    outtake.releaseSecondary();
                    sleep(1000);
                    encoderDrive(2,0.2);
                    lift.moveLift(.7f);
                    sleep(200);
                    lift.moveLift(0f);
                    outtake.pivotStart();
                    strafeDrive(36,-0.5);
                    sleep(500);
                    encoderDrive(5,-0.7);
                    break;
                case MIDDLE:
                    encoderDrive(33, -.4);
                    sleep(1000);
                    turn(85, .3);
                    sleep(1000);
                    encoderDrive(4, -.4);
                    sleep(1000);
                    outtakePixel();
                    sleep(1000);
                    encoderDrive(27, -.5);
                    sleep(1000);
                    strafeDrive(10, -.4);
                    sleep(1000);
                    lift.moveLift(-.75f);
                    sleep(200);
                    lift.moveLift(0f);
                    sleep(1000);
                    outtake.pivotEnding();
                    sleep(2000);
                    outtake.releaseMain();
                    outtake.releaseSecondary();
                    sleep(500);
                    encoderDrive(3, .4);
                    sleep(500);
                    outtake.pivotStart();
                   sleep(300);
                    sleep(1000);
                  strafeDrive(28, -.6);
                  encoderDrive(10, -.4);

                    break;
                case LEFT:
                    strafeDrive(6, .4);
                    sleep(1000);
                    encoderDrive(42, -.4);
                    sleep(1000);
                    strafeDrive(15, -.4);
                    sleep(1000);
                    encoderDrive(14, .4);
                    sleep(1000);
                    encoderDrive(14, -.4);
                     sleep(1000);
                     outtakePixel();
                     sleep(1000);
                      strafeDrive(25, -.4);
                     sleep(750);
                    turn(82, .3);
                    sleep(1000);
                    encoderDrive(11.75, -.5);
                    sleep(1000);
                    strafeDrive(24, -.4);
                    sleep(1000);
                    encoderTurn(2, .4);
                    sleep(1000);
                    strafeDrive(5, -.4);
                    sleep(1000);
                    encoderDrive(2, -.4);
                    sleep(1000);
                    lift.moveLift(-.75f);
                    sleep(170);
                    lift.moveLift(0f);
                    sleep(1000);
                    outtake.pivotEnding();
                    sleep(1700);
                    outtake.releaseSecondary();
                    sleep(1000);
                    encoderDrive(2,0.5);
                    sleep(500);
                    outtake.pivotStart();
                    strafeDrive(15,-1);
                    lift.moveLift(.75f);
                    sleep(200);
                    lift.moveLift(0f);
                    outtake.pivotStart();






                   break;
            }

            sleep(5000);

            sleep(30000);

        }
        driveTrain.die();
    }

    public void encoderDrive(double inches, double speed) {
        driveTrain.resetEncoder();
        driveTrain.setPower(speed);
        int counts = (int) (inches * COUNTS_PER_INCH);
        while (opModeIsActive()) {
            for (int position : driveTrain.getWheelPosition()) {
                if (Math.abs(position) > counts) {
                    driveTrain.die();
                    return;
                }
            }
        }
        driveTrain.die();
        driveTrain.resetEncoder();
    }

    public void encoderTurn(double inches, double speed) {
        driveTrain.resetEncoder();
        driveTrain.setTurnPower(speed);
        int counts = (int) (inches * COUNTS_PER_INCH);
        while (opModeIsActive()) {
            for (int position : driveTrain.getWheelPosition()) {
                if (Math.abs(position) > counts) {
                    driveTrain.die();
                    return;
                }
            }
        }
        driveTrain.die();
        driveTrain.resetEncoder();
    }

    public void strafeDrive(double inches, double speed) {
        driveTrain.resetEncoder();

        driveTrain.setStrafePower(speed);
        int counts = (int) (inches * COUNTS_PER_INCH);
        while (opModeIsActive()) {
            for (int position : driveTrain.getWheelPosition()) {
                if (Math.abs(position) > counts) {
                    driveTrain.die();
                    return;
                }
            }
        }
        driveTrain.die();
        driveTrain.resetEncoder();
    }


    public void turn(double angle, double speed) {

        driveTrain.resetHeading();
        double error = angle;
        while (opModeIsActive() && Math.abs(error)>2){
            double power = (error < 0? speed: -speed);
            driveTrain.setTurnPower(power);
            error = angle-driveTrain.getHeading();
            telemetry.addData("Error: ", error);
            telemetry.update();
        }
        driveTrain.die();
    }
    // outtake purple pixel
    public void outtakePixel() {
        intake.dropOff();
        sleep(1000);
        intake.die();
    }
}

