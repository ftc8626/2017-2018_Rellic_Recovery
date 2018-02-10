package org.firstinspires.ftc.teamcode.Auto.Jewel_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware_Speedy;

/**
 * Created by kpingel on 11/10/17.
 */
@Autonomous(name="Red_Right_Jewel_Autonomous", group="Jewel")
public class Red_Right_Jewel_Autonomous extends LinearOpMode {

    Hardware_Speedy robot = new Hardware_Speedy();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;
    // static final double DRIVE_GEAR_REDUCTION = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //new variable for encoders
    static final double ENCODER_DRIVE = (COUNTS_PER_INCH) * (.8);

    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        boolean LEDState = true;

        // robot.rightColorSensor.enableLed(LEDState);//moved here from below


        float hsvValues[] = {0F, 0F, 0F};
        //       final float values[] = hsvValues;
        // the above line may have been the cause of the init problem

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        //path starts here

        robot.rightIntakeArm.setPosition(.7);
        robot.leftIntakeArm.setPosition(.7);
        robot.leftBottomGrabber.setPosition(.1);
        robot.rightBottomGrabber.setPosition(.1);
        robot.liftMotor.setPower(.3);
        sleep(1000);
        robot.liftMotor.setPower(0);

        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.5);
        robot.rightJewel.setPosition(.4);
        robot.rightJewel.setPosition(.3);
        sleep(2000);

        while (opModeIsActive() && robot.rightColorSensor.blue() < 1 && robot.rightColorSensor.red() < 1 && getRuntime() < 10) {
            robot.rightColorSensor.enableLed(LEDState);

            telemetry.addData("2 Clear", robot.rightColorSensor.alpha());
            telemetry.addData("3 Red  ", robot.rightColorSensor.red());
            telemetry.addData("4 Green", robot.rightColorSensor.green());
            telemetry.addData("5 Blue ", robot.rightColorSensor.blue());
            telemetry.addData("6 Hue", hsvValues[0]);
            telemetry.addData("time", getRuntime());
            telemetry.update();

           /* if (robot.rightColorSensor.red() >= 1) {
                encoderDrive(DRIVE_SPEED, 2, -2, 4);
                sleep(500);
                robot.rightJewel.setPosition(1);
                sleep(500);
                encoderDrive(DRIVE_SPEED, -2, 2, 4.0);
            }*/
        }

        if (robot.rightColorSensor.blue() > robot.rightColorSensor.red() /*&& robot.rightColorSensor.blue() > robot.rightColorSensor.green()*/) {
            encoderDrive(DRIVE_SPEED, 2, -2, 4);
            sleep(500);
            robot.rightJewel.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, -2, 2, 4);
        }

        if (robot.rightColorSensor.red() > robot.rightColorSensor.blue()/* && robot.rightColorSensor.red() > robot.rightColorSensor.green()*/) {
            encoderDrive(DRIVE_SPEED, -2, 2, 4);
            sleep(500);
            robot.rightJewel.setPosition(1);
            sleep(500);
            encoderDrive(DRIVE_SPEED, 2, -2, 4);
        }

        robot.rightJewel.setPosition(1);
        sleep(500);
        encoderDrive(.4, 32, 32, 5.0);
        robot.liftMotor.setPower(-.3);
        sleep(500);
        robot.liftMotor.setPower(0);
        encoderDrive(DRIVE_SPEED, -5, 5, 4.0);
        encoderDrive(DRIVE_SPEED, 16, 16, 5.0);
        robot.leftBottomGrabber.setPosition(.38);
        robot.rightBottomGrabber.setPosition(.41);
        sleep(500);
        encoderDrive(DRIVE_SPEED, -5, -5, 5.0);
        sleep(500);//
        encoderDrive(DRIVE_SPEED, 5, 5, 5);//
        sleep(500);//
        encoderDrive(DRIVE_SPEED, -5, -5, 5);//

        telemetry.addData("Path", "Complete");
        telemetry.update();

    }




    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int) (leftInches * ENCODER_DRIVE);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int) (rightInches * ENCODER_DRIVE);
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}