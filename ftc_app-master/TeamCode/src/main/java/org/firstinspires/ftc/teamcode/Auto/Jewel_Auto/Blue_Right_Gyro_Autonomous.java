package org.firstinspires.ftc.teamcode.Auto.Jewel_Auto;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Hardware_Speedy;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by kpingel on 11/10/17.
 */
@Autonomous(name="BlueRightGyroAutonomous", group="Jewel")
@Disabled
public class Blue_Right_Gyro_Autonomous extends LinearOpMode {

    Hardware_Speedy robot = new Hardware_Speedy();
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;
    // static final double DRIVE_GEAR_REDUCTION = 2.0 ;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);
    //new variable for encoders
    static final double ENCODER_DRIVE = (COUNTS_PER_INCH) * (.8);

//    static final double DRIVE_SPEED = 0.6;
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.04;     // Larger is more responsive, but also less stable

    static final String LEFT = "LEFT";
    static final String CENTER = "CENTER";
    static final String RIGHT = "RIGHT";

    static final Double ANGLE_LEFT = new Double(90);
    static final Double ANGLE_CENTER = new Double(120);
    static final Double ANGLE_RIGHT = new Double(150);
    static final Double DISTANCE_LEFT = new Double(12);
    static final Double DISTANCE_CENTER = new Double(14);
    static final Double DISTANCE_RIGHT = new Double(16);

    VuforiaLocalizer vuforia;

    Map<String, Double> scoringAngleMap = null;
    Map<String, Double> scoringDistanceMap = null;

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro gyro;

    @Override
    public void runOpMode() {

        scoringAngleMap = buildScoringAngleMap();
        scoringDistanceMap = buildScoringDistanceMap();

        robot.init(hardwareMap);

        // The following reversed motor directions is needed for gyroDrive!
        robot.leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        gyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        gyro.calibrate();

        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        boolean LEDState = true;

        // robot.rightColorSensor.enableLed(LEDState);//moved here from below

//        float hsvValues[] = {0F, 0F, 0F};
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
        //read pictograph
        RelicRecoveryVuMark vuMark = readPictograph();
        String column = vuMark.toString();
        double scoringDistance = scoringDistanceMap.get(column);
        double scoringAngle = scoringAngleMap.get(column);

        // raise lift
        closeGrabber();
        raiseLift();

        //lower jewel arm
        lowerJewelArm();
        sleep(2000);

        //sense color
        while (opModeIsActive() && robot.rightColorSensor.blue() < 1 && robot.rightColorSensor.red() < 1 && getRuntime() < 10) {
            robot.rightColorSensor.enableLed(LEDState);

            reportColorSensor();
            telemetry.update();

        }

        //if sensor reads blue, knock off red (backward)
        if (robot.rightColorSensor.blue() > robot.rightColorSensor.red()) {
            encoderDrive(DRIVE_SPEED, -2, 2, 4);
            raiseJewelArm();
            encoderDrive(DRIVE_SPEED, 2, -2, 4);
        }

        //if sensor reads red, knock it off (forward)
        if (robot.rightColorSensor.red() > robot.rightColorSensor.blue()) {
            encoderDrive(DRIVE_SPEED, 2, -2, 4);
            raiseJewelArm();
            encoderDrive(DRIVE_SPEED, -2, 2, 4);
        }

        robot.rightJewel.setPosition(1);
        //gyroDrive(DRIVE_SPEED, 12, 0);//temporary line
//        encoderDrive(DRIVE_SPEED/*.4*/, -31, -31, 5.0);
//        gyroDrive(DRIVE_SPEED/*.4*/, 31, 0.0);
        gyroDrive(DRIVE_SPEED/*.4*/, 62, 0.0);
        //encoderDrive(DRIVE_SPEED, 17, -17, 4.0);
        gyroTurn(DRIVE_SPEED, 120);//temporary line
        //gyroTurn(DRIVE_SPEED, scoringAngle);
        //encoderDrive(DRIVE_SPEED, 15, 15, 5.0);
        gyroDrive(DRIVE_SPEED, scoringDistance, 0);
        openGrabber();
        sleep(500);
        //encoderDrive(DRIVE_SPEED, -2, -2, 5.0);
        //gyroDrive(DRIVE_SPEED, -2, 0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private Map<String,Double> buildScoringAngleMap() {
        Map<String, Double> scoringAngleMap = new HashMap<String, Double>();

        scoringAngleMap.put(LEFT, ANGLE_LEFT);
        scoringAngleMap.put(CENTER, ANGLE_CENTER);
        scoringAngleMap.put(RIGHT, ANGLE_RIGHT);

        return scoringAngleMap;
    }

    private Map<String,Double> buildScoringDistanceMap() {
        Map<String, Double> scoringDistanceMap = new HashMap<String, Double>();

        scoringDistanceMap.put(LEFT, DISTANCE_LEFT);
        scoringDistanceMap.put(CENTER, DISTANCE_CENTER);
        scoringDistanceMap.put(RIGHT, DISTANCE_RIGHT);

        return scoringDistanceMap;
    }

    private void raiseLift() {
        robot.liftMotor.setPower(.3);
        sleep(1000);
        robot.liftMotor.setPower(0);
    }

    private void raiseJewelArm() {
        sleep(500);
        robot.rightJewel.setPosition(1);
        sleep(500);
    }

    private RelicRecoveryVuMark readPictograph() {
        //call vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.CENTER;
        return vuMark;
    }

    private void lowerJewelArm() {
        robot.rightJewel.setPosition(.9);
        robot.rightJewel.setPosition(.8);
        robot.rightJewel.setPosition(.7);
        robot.rightJewel.setPosition(.6);
        robot.rightJewel.setPosition(.5);
        robot.rightJewel.setPosition(.4);
        robot.rightJewel.setPosition(.3);
    }

    private void reportColorSensor() {
        float hsvValues[] = {0F, 0F, 0F};
        telemetry.addData("2 Clear", robot.rightColorSensor.alpha());
        telemetry.addData("3 Red  ", robot.rightColorSensor.red());
        telemetry.addData("4 Green", robot.rightColorSensor.green());
        telemetry.addData("5 Blue ", robot.rightColorSensor.blue());
        telemetry.addData("6 Hue", hsvValues[0]);
        telemetry.addData("time", getRuntime());
    }

    private void closeGrabber() {
        robot.leftBottomGrabber.setPosition(.1);
        robot.rightBottomGrabber.setPosition(.1);
    }

    private void openGrabber() {
        robot.leftBottomGrabber.setPosition(.38);
        robot.rightBottomGrabber.setPosition(.41);
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

            int i = 0;
            telemetry.addData("before loop", i);
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
//                        robot.leftFrontMotor.getCurrentPosition(),
//                        robot.rightFrontMotor.getCurrentPosition());
                reportEncoderValues(robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                telemetry.addData("loop count:", i);
                telemetry.update();
                i++;
            }
        }
    }

    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            reportEncoderValues(robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
            reportMotorStatus(robot.leftFrontMotor.isBusy(), robot.rightFrontMotor.isBusy());
            telemetry.update();

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);

            int i = 0;
            telemetry.addData("before loop", i);
            reportMotorStatus(robot.leftFrontMotor.isBusy(), robot.rightFrontMotor.isBusy());
            telemetry.update();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                     (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFrontMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.addData("loop count:", i);
                reportEncoderValues(robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
                reportMotorStatus(robot.leftFrontMotor.isBusy(), robot.rightFrontMotor.isBusy());
                telemetry.update();
                i++ ;
            }

            reportEncoderValues(robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
            reportMotorStatus(robot.leftFrontMotor.isBusy(), robot.rightFrontMotor.isBusy());
            telemetry.update();

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            reportEncoderValues(robot.leftFrontMotor.getCurrentPosition(), robot.rightFrontMotor.getCurrentPosition());
            reportMotorStatus(robot.leftFrontMotor.isBusy(), robot.rightFrontMotor.isBusy());
            telemetry.update();
        }
    }

    private void reportMotorStatus(boolean leftMotorStatus, boolean rightMotorStatus) {
        telemetry.addData("MotorStatus",  "%s:%s", new Boolean(leftMotorStatus), new Boolean(rightMotorStatus));
    }

    private void reportEncoderValues(int leftEncoder, int rightEncoder) {
        telemetry.addData("Actual",  "%7d:%7d", leftEncoder, rightEncoder);
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//        telemetry.addData(">", "left encoder = %d", robot.leftFrontMotor.getCurrentPosition());
//        telemetry.addData(">", "right encoder = %d", robot.rightFrontMotor.getCurrentPosition());

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}