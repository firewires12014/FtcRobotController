package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;


@Autonomous(name = "autoTest", group = "Robot")
public class autoTest extends LinearOpMode {
    DriveTrain driveTrain;
    Intake intake;
    Outtake outtake;

    static final double COUNTS_PER_MOTOR_REV = 537.7; // <----- this is probably wrong. check it.
    static final double DRIVE_GEAR_REDUCTION = 1.0; // <----- fiddle with this. test until the robot goes 12 inches with encDrive
    static final double WHEEL_DIAMETER_INCHES = 3.78; // <----- this is right
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake.resetIntake();
        outtake.lockPixels();

        waitForStart();
        while (opModeIsActive()) {
            encoderDrive(25, -.4);
            sleep(1000);
            turn(-87, .3);

            sleep(5000);

            sleep(30000);

        }
        driveTrain.die();
    }

    public void encoderDrive(double inches, double speed) {

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
        sleep(2000);
        intake.die();
    }
    }

