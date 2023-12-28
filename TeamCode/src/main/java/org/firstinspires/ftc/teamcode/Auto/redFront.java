package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Vision.CenterstageDetectorRed;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "redFront", group = "Robot")
public class redFront extends LinearOpMode {
    public OpenCvCamera camera;
    private CenterstageDetectorRed CenterstageDetectorRed = new CenterstageDetectorRed(telemetry); // camera stuff
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
        camera.setPipeline(CenterstageDetectorRed); // this is what gets the camera going.
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

            CenterstageDetectorRed.Location location = CenterstageDetectorRed.getLocation();
            telemetry.addData("Location: ", location);
            telemetry.update();
        }
        outtake.lockSecondary();
        outtake.releaseMain();
        waitForStart();

        while (opModeIsActive()) {
            CenterstageDetectorRed.Location location = CenterstageDetectorRed.getLocation();
            switch (location) {
                case LEFT:
                    encoderDrive(27, -.4);
                    sleep(1000);
                    turn(-87, .3);
                    sleep(1000);
                    encoderDrive(6, .3);
                    sleep(500);
                    encoderDrive(3, -.4);
                    sleep(1000);
                    outtakePixel();
                    sleep(1000);
                    strafeDrive(25,-0.75);
                    sleep(500);
                    break;
                case MIDDLE:
                   strafeDrive(5,-0.4);
                    encoderDrive(47,-0.5);
                    sleep(750);
                    outtakePixel();
                    sleep(500);
                    encoderDrive(3.75,-0.5);
                    sleep(500);
                    turn(-87,0.4);
                    sleep(500);
                    encoderDrive(70,-0.5);
                    sleep(500);
                    strafeDrive(49,0.4);
                    sleep(500);
                    encoderDrive(11,-0.4);
                    sleep(500);
                    encoderTurn(0.5,0.4);
                    sleep(500);
                    lift.moveLift(-.75f);
                    sleep(200);
                    lift.moveLift(0f);
                    encoderDrive(1,1);
                    sleep(1000);
                    outtake.pivotEnding();
                    sleep(2000);
                    outtake.releaseSecondary();
                    sleep(1000);
                    outtake.pivotStart();
                    lift.moveLift(.75f);
                    sleep(200);
                    lift.moveLift(0f);
                    strafeDrive(20,-0.75);
                    sleep(500);
                    break;
                case RIGHT:
                    encoderDrive(2, -.4);
                    strafeDrive(39, .6);
                    sleep(1000);
                    encoderDrive(20, -.4);
                    turn(-87, .3);
                    encoderDrive(9, .4);
                    sleep(1000);
                    encoderDrive(2.5, -.4);
                    outtakePixel();
                    encoderDrive(1.5, .4);
                    sleep(1000);
                    strafeDrive(15,-0.75);
                    sleep(500);

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
    }

