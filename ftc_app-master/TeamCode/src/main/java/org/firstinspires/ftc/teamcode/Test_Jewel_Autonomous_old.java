package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kpingel on 11/10/17.
 */
@Autonomous(name="Test_Jewel_Autonomous", group="Speedy")
@Disabled
public class Test_Jewel_Autonomous_old extends LinearOpMode {

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

    public final String BALANCE_BOARD = "BALANCE_BOARD";
    public final String BALANCE_BOARD_LEFT = "BALANCE_BOARD_LEFT";
    public final String BALANCE_BOARD_RIGHT = "BALANCE_BOARD_RIGHT";

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftFrontMotor.getCurrentPosition(),
                robot.rightFrontMotor.getCurrentPosition());
        telemetry.update();

        waitForStart();

        robot.rightJewel.setPosition(.35);
        sleep(500);
        while (robot.rightColorSensor.red() < 1) {

            robot.rightColorSensor.enableLed(true);    //Set the mode of the color sensor using LEDState


            telemetry.addData("6 Red2  ", robot.rightColorSensor.red());
            telemetry.addData("8 Blue2 ", robot.rightColorSensor.blue());

            telemetry.update();

            if (robot.rightColorSensor.blue() > robot.rightColorSensor.red() && robot.rightColorSensor.blue() > 1) {
                encoderDrive(DRIVE_SPEED, 3, 3, 5);
                encoderDrive(DRIVE_SPEED, -3, -3, 5);
            }
        }
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