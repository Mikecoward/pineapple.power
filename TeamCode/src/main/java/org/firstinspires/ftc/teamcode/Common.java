package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Method;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import android.util.Log;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.LED;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode2.ColorSensorV31;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLStatus;
import java.util.ArrayList;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoController;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
//import org.firstinspires.ftc.teamcode2.ColorSensorV31;

import java.lang.Math;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.hardware.LED;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.function.Supplier;




class Common {
    static DcMotor  leftFrontDrive   = null;
    static DcMotor  rightFrontDrive  = null;
    static DcMotor  leftBackDrive    = null;
    static DcMotor  rightBackDrive   = null;


    static Limelight3A limelight;
    static IMU imu = null;

    static GoBildaPinpointDriver odo;
    static Pose2D ppPos, ppVel, targetPos;
    static int ppPosAbsolute = 0;  // 1Number of updates.  0 is relative, 1+ is absolute
    static int ppPosThreshV = 0;    // Velocity blocked
    static int ppPosThreshS = 0;    // Fiducial size blocked

    static DcMotor intaking = null;
    static DcMotor shoot = null;

    static DcMotor shoot2 = null;

    static DcMotor lifting = null;

    static Telemetry telemetry = null;
    static Gamepad gamepad1 = null;
    static Gamepad gamepad2 = null;

    static Servo radvance = null;
    static Servo madvance = null;
    static Servo ladvance = null;
    static CRServo advancewheel = null;

    // Run motor slowly downwards until current gets too high, then decide that this must be the zero point.
    static void zeroBothMotors() {
        //. we don't need anything here yet
        // Only zero a motor if the mechanism has a known physical reference point that the motor can safely move into.

    }

    static void configRobot(HardwareMap hardwareMap, boolean recalibrateIMU) {

        configurePinpoint(hardwareMap, recalibrateIMU);

        scalefactor = 2800.0;
        // servos
        radvance = hardwareMap.servo.get("radvance");
        madvance = hardwareMap.servo.get("madvance");
        ladvance = hardwareMap.servo.get("ladvance");
        ladvance.setDirection(Servo.Direction.REVERSE);

        advancewheel = hardwareMap.crservo.get("advancewheel");
        advancewheel.setDirection(CRServo.Direction.REVERSE);

        intaking  = hardwareMap.dcMotor.get("intake");
        intaking.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intaking.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intaking.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shoot  = hardwareMap.dcMotor.get("shoot");
        shoot.setDirection(DcMotor.Direction.FORWARD);
        shoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shoot2  = hardwareMap.dcMotor.get("shoot2");
        shoot.setDirection(DcMotor.Direction.FORWARD);
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lifting = hardwareMap.dcMotor.get("lifting");
        lifting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive  = hardwareMap.dcMotor.get("leftFront");
        leftBackDrive   = hardwareMap.dcMotor.get("leftBack");
        rightFrontDrive = hardwareMap.dcMotor.get("rightFront");
        rightBackDrive  = hardwareMap.dcMotor.get("rightBack");

        //1 making all of these set velocity
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
        Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

    }

    static boolean initialPositionSet = false;
    // Sets position in the Pinpoint IMU
    static void setInitialPosition(double X, double Y, double H) {
        odo.setPosition(new Pose2D(
                DistanceUnit.MM,
                X,
                Y,
                AngleUnit.DEGREES,
                H
        ));
    }

    static void tempMT2() {
         // Temporarily set ppPos so limelight doesn't crash
        ppPos = new Pose2D(DistanceUnit.MM,
                0,
                0,
                AngleUnit.DEGREES,
                90);

        checkLimelight();

        if (isMt2Valid()) {
            telemetry.addLine("Valid");
            odo.setPosition(new Pose2D(DistanceUnit.INCH,
                getMt2X(),
                getMt2Y(),
                AngleUnit.DEGREES,
                90));
            telemetry.addLine(getMt2X()+", "+getMt2Y());
            ppPosAbsolute++;
            //updatePinpoint();

        } else {
            telemetry.addLine("not Valid");
            odo.setPosition(new Pose2D(DistanceUnit.MM,
                0,
                0,
                AngleUnit.DEGREES,
                90));
        }
    }


    static void updatePinpoint() {
            odo.update();

            ppPos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f, A: %d, V: %d, F: %d}", ppPos.getX(DistanceUnit.INCH), ppPos.getY(DistanceUnit.INCH), normalizeAngleD(ppPos.getHeading(AngleUnit.DEGREES)-90), ppPosAbsolute, ppPosThreshV, ppPosThreshS);
            telemetry.addData("Position", data);


            //gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
            odo.update();

            double vx = odo.getVelX(DistanceUnit.MM);
            double vy = odo.getVelY(DistanceUnit.MM);
            double vh = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            telemetry.addData("Velocity",
                    String.format(Locale.US, "{XVel: %.2f, YVel: %.2f, HVel: %.2f}", vx, vy, vh));


            /*
             Gets the Pinpoint device status. Pinpoint can reflect a few states:
               READY: device working normally
               CALIBRATING: IMU calibrating
               NOT_READY: resetting after power cycle
               FAULT_NO_PODS_DETECTED
               FAULT_X_POD_NOT_DETECTED
               FAULT_Y_POD_NOT_DETECTED
            */

            telemetry.addData("Status", odo.getDeviceStatus());


    }

    static void configurePinpoint(HardwareMap hardwareMap, boolean recalibrateIMU) {

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odo.setOffsets(-98.0, 150.0, DistanceUnit.MM);

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */

        if (recalibrateIMU) {
            odo.recalibrateIMU();
            sleep(500);
            odo.resetPosAndIMU();
            sleep(500);
        }

    }

    static boolean isMt2Valid() {
        return botpose_mt2!=null;
    }


    static double getMt2X() {
        return botpose_mt2.getPosition().x/.0254;
    }

    static double getMt2Y() {
        return botpose_mt2.getPosition().y/.0254;
    }

    static Pose3D botpose_mt2;

    static void checkLimelight() {
        double FIDUCIAL_THRESHOLD_MT2 = 0.0035;

        boolean mt2_valid = false;

        LLStatus status = limelight.getStatus();
        /*telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());f
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());
           */
            // First, tell Limelight which way your robot is facing
            double robotYaw = ppPos.getHeading(AngleUnit.DEGREES);

            /*
            if (!redAlliance) {
                robotYaw = normalizeAngleD(robotYaw+180);
            }
             */

            //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - initialYaw + llYawOffset;

            limelight.updateRobotOrientation(robotYaw);

            LLResult result = limelight.getLatestResult();
            botpose_mt2 = null;

            if (result != null && result.isValid()) {
                            }

            if (result != null) {

                if (result.isValid()) {
                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f, ta: %.4f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees(), fr.getTargetArea());
                        if (fr.getTargetArea()>FIDUCIAL_THRESHOLD_MT2) {
                            mt2_valid = true;
                        } else {
                            ppPosThreshS++;
                        }
                    }


                    if (mt2_valid) { // If we're close enough to ficucial to trust it:
                        botpose_mt2 = result.getBotpose_MT2();
                        if (botpose_mt2 != null) {
                            /*
                            if (!redAlliance) { // Flip coordinates if on blue team
                                Position flippedPose = new Position(botpose_mt2.getPosition().unit,-botpose_mt2.getPosition().x, -botpose_mt2.getPosition().y, botpose_mt2.getPosition().z, botpose_mt2.getPosition().acquisitionTime);
                                botpose_mt2 = new Pose3D(flippedPose, botpose_mt2.getOrientation());
                            }

                             */

                            telemetry.addData("MT2 Location:", "(" + botpose_mt2.getPosition().x/.0254 + "in, " + botpose_mt2.getPosition().y/.0254 + "in)");

                            telemetry.addData("Yaw", robotYaw);
                        }
                    }

                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                    for (LLResultTypes.BarcodeResult br : barcodeResults) {
                        telemetry.addData("Barcode", "Data: %s", br.getData());
                    }

                    // Access classifier results
                    List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                    for (LLResultTypes.ClassifierResult cr : classifierResults) {
                        telemetry.addData("Classifier", "Class: %s, Confidence: %.2f", cr.getClassName(), cr.getConfidence());
                    }

                    // Access detector results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }
    }

    // Helper method to set motor powers on your specific robot
    /*static void setMotorPowersPF(MotorPowers powers) {
        ((DcMotorEx) leftFrontDrive).setPower(powers.frontLeft);
        ((DcMotorEx) leftBackDrive).setPower(powers.backLeft);
        ((DcMotorEx) rightFrontDrive).setPower(powers.frontRight);
        ((DcMotorEx) rightBackDrive).setPower(powers.backRight);
    }*/

    static void checkAlliance(HardwareMap hardwareMap) {
    /*   TouchSensor touchSensor1;  // Touch sensor Object
       touchSensor1 = hardwareMap.get(TouchSensor.class, "switch1");

         if (touchSensor1.isPressed()) {
             redAlliance = false;
         } else {
             redAlliance = true;
         }
         */
    }

     static double normalizeAngleD(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // total velocity in inches/sec
    static double robotVelocity() {
        double xVel = ppVel.getX(DistanceUnit.INCH);
        double yVel = ppVel.getY(DistanceUnit.INCH);
        return Math.sqrt(xVel * xVel + yVel * yVel);
    }

    // total velocity in degrees/sec
    static double robotAngularVelocity() {
        return ppVel.getHeading(AngleUnit.DEGREES);
    }

    static boolean updatePosFromApril = false;
    // Update position from April tags if robot not moving
    static void updatePos() {
        if (!updatePosFromApril)
            return;
        double UPDATE_VELOCITY_THRESHOLD = 5;
        double UPDATE_ANGULAR_V_THRESHOLD = 5;
        /*if (isMt2Valid()) {
            if (robotVelocity()<UPDATE_VELOCITY_THRESHOLD && robotAngularVelocity()<UPDATE_ANGULAR_V_THRESHOLD) {
                /*odo.setPositionXYonly(new Pose2D(DistanceUnit.INCH,
                    getMt2X(),
                    getMt2Y(),
                    AngleUnit.DEGREES,
                    ppPos.getHeading(AngleUnit.DEGREES)));  // Angle should be ignored
                ppPosAbsolute++;
                //updatePinpoint();
            } else {
                ppPosThreshV++;
            }

        }*/
}


    static double error_x, error_y, error_h, scalefactor;

    static void updateMotors() {
        // Have current position in ppPos
        // Have current Velocity in ppVel
        // Have target position in targetPos

        double VEL_SCALE_FACTOR = 20;
        double ANG_SCALE_FACTOR = 20;

        error_x = targetPos.getX(DistanceUnit.INCH) - (ppPos.getX(DistanceUnit.INCH)+ppVel.getX(DistanceUnit.INCH)/VEL_SCALE_FACTOR);
        error_y = targetPos.getY(DistanceUnit.INCH) - (ppPos.getY(DistanceUnit.INCH)+ppVel.getY(DistanceUnit.INCH)/VEL_SCALE_FACTOR);
        error_h = normalizeAngleD(targetPos.getHeading(AngleUnit.DEGREES) - ((ppPos.getHeading(AngleUnit.DEGREES)-90)+ppVel.getHeading(AngleUnit.DEGREES)/ANG_SCALE_FACTOR));

        double current_heading_r = Math.toRadians(ppPos.getHeading(AngleUnit.DEGREES));
        double y = -error_y / 20;
        double x = error_x / 20;
        double rx = -error_h / 30;

        double rotX = y * Math.cos(-current_heading_r) - x * Math.sin(-current_heading_r);
        double rotY = y * Math.sin(-current_heading_r) + x * Math.cos(-current_heading_r);
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX -   rx) / denominator;

        ((DcMotorEx) leftFrontDrive).setVelocity(scalefactor * frontLeftPower);
        ((DcMotorEx) leftBackDrive).setVelocity(scalefactor * backLeftPower);
        ((DcMotorEx) rightFrontDrive).setVelocity(scalefactor * frontRightPower);
        ((DcMotorEx) rightBackDrive).setVelocity(scalefactor * backRightPower);
    }

    static void stopMotors() {

        ((DcMotorEx) leftFrontDrive).setVelocity(0);
        ((DcMotorEx) leftBackDrive).setVelocity(0);
        ((DcMotorEx) rightFrontDrive).setVelocity(0);
        ((DcMotorEx) rightBackDrive).setVelocity(0);
    }

    static double getRuntime() {
        return (System.currentTimeMillis()/1000.0);
    }

    static void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


}
