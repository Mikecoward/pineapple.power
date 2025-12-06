package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode2.ColorSensorV31;
//import org.firstinspires.ftc.teamcode2.GoBildaPinpointDriver;

import java.lang.Math;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;
import com.qualcomm.robotcore.hardware.LED;
// stuff for the odometry (imports taken from gpt)
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import static org.firstinspires.ftc.teamcode.Common.limelight;
import static org.firstinspires.ftc.teamcode.Common.normalizeAngleD;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.goBack;

import java.util.List;
import java.util.function.Supplier;

/* Control Map

Gamepad A:
Right Stick X: move left/right
Right Stick Y: move forward/back
Left Stick X:  turn robot left/right
Left Stick Y:
Press Right/Left Stick in:

Dpad L/R:
Dpad Up/Down:
A button: increases spinner angle by 60 degrees,
 B button:   push once- hood moves up a little, push again, hood goes down
X button:    while holding, shooter motor is set to max power
Y button: start spinning intake
L button: make kicker go up/down
L trigger:
R button:
R trigger:
Logo (Logitech Button):
back button:
start button:  Set IMU back to 0.

Gamepad 2:
Right Stick X:
Right Stick Y:
Left Stick X:
Left Stick Y:
Dpad L:
Dpad R:
Dpad Up:
Dpad Down:

A button:
B button:
X button:
Y button:
L button:
L trigger:
R button:
R trigger:
Logo (Logitech Button):
back button:
start button:

*/

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name="AS-first robot-3", group="Robot")

public class PpBot extends LinearOpMode {

    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean toggle23 = false;

    boolean swapDirections = false;
    boolean debounceDirection = false;

    IMU imu;

    double DRIVE_TICKS_PER_SEC_MAX = 2800.0;

    private void MyInit() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 1.0))
                .build();
    }

    private void MyStart() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void runOpMode() {


        char colorf = 'n';
        char colorb = 'n';
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double speed = 0;
        boolean n_color_override = true;
        double x = 0;
        double y = 0;
        double rx = 0;

        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        Common.telemetry = telemetry;
        if (Common.initialPositionSet) {
            Common.configRobot(hardwareMap, false);
            telemetry.addLine("Taking position from previous run");

        } else {
            Common.configRobot(hardwareMap, true);
            Common.setInitialPosition(-31.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
            telemetry.addLine("Setting position to -31.5, -63.5, 0");
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        MyInit();     // <-- Add this
        MyStart();    // <-- Add this

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
/*
        Common.updatePinpoint();

        Common.zeroBothMotors();
        /*
        /* Run until the driver presses stop */
        boolean lastA = false;
        float rightTrigger = 0;
        float leftTrigger = 0;
        boolean rightbumperpressed = false;
        boolean leftbumperpressed = false;

        double curAngle = 62; // current angle
        //Common.Spinner.setPosition(curAngle/360);

        //boolean hoodUp = false; // starts down
        boolean kickerUp = false;
        long lastKickTime = 0;
        Pose2D pos = Common.ppPos;

        // At top of OpMode
        boolean lastB = false;
        boolean hoodUp = false;

        double moveAmount = 24 * 360;      // degrees per press (2 rotations)
        double incrementAmount = 60; // degrees per press (2 rotations)






        final double TURN_SPEED = 0.25; // Slower speed for alignment
        final double TX_DEADBAND = 2.0; // Target is "aligned" if tx is within +/- 1.0 degrees


        Pose2D ppPos = Common.odo.getPosition();



        goBack goBack1 = new goBack(follower); // pass your TeleOp's follower

        boolean goingBack = false;

        while (opModeIsActive()) {

            ((DcMotorEx) Common.intaking).setVelocity(1000);

            ppPos = Common.odo.getPosition(); // ppPos now holds X, Y, heading
            double currentX = ppPos.getX(DistanceUnit.INCH); // current X in inches
            double currentY = ppPos.getY(DistanceUnit.INCH); // current Y in inches


            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

            Common.updatePinpoint();


            if (gamepad2.start) {
                Common.zeroBothMotors();
            }


            if (gamepad1.aWasPressed()) {
                follower.followPath(pathChain.get());
                automatedDrive = true;
            }
            //Stop automated following if the follower is done
            if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
                follower.startTeleopDrive();
                automatedDrive = false;
            }


            if (goingBack) {

            }

            if (gamepad1.b) {
                telemetry.addLine("Pressed B");
            }
            if (gamepad1.x) {
                telemetry.addLine("Pressed X");
            }
            if (gamepad1.y) {
                telemetry.addLine("Pressed Y");
            }

            if (gamepad1.start) {
                Common.setInitialPosition(0, 0, 0);
            }

            handleJoystick();
            telemetry.update();
        }
    }

    private void handleJoystick() {
        // Handle Joysticks


        double stickY = -gamepad1.right_stick_y;
        double stickX = gamepad1.right_stick_x;
        double stickR = gamepad1.left_stick_x/1.5;

        // Reverse Sticks


        double minfactor = 0.05;
        double powerfactor = 1.200;
        // Expo control:
        if (!(stickX == 0)) {
            stickX = stickX * Math.pow(Math.abs(stickX), powerfactor-1)+ (stickX/Math.abs(stickX)*minfactor);
        }
        if (!(stickY == 0)) {
            stickY = stickY * Math.pow(Math.abs(stickY), powerfactor-1)+ (stickY/Math.abs(stickY)*minfactor);
        }
        if (!(stickR == 0)) {
            stickR = stickR * Math.pow(Math.abs(stickR), powerfactor-1)+ (stickR/Math.abs(stickR)*minfactor);
        }
        stickX = stickX * 1.1; // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        boolean headingfield = true;
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        if (headingfield){

            double rotX = stickX * Math.cos(-botHeading) - stickY * Math.sin(-botHeading);
            double rotY = stickX * Math.sin(-botHeading) + stickY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            frontLeftPower = (rotY + rotX + stickR);
            backLeftPower = (rotY - rotX + stickR);
            frontRightPower = (rotY - rotX - stickR);
            backRightPower = (rotY + rotX - stickR);

        }
        else{
            frontLeftPower = (stickY + stickX + stickR);
            backLeftPower = (stickY - stickX + stickR);
            frontRightPower = (stickY - stickX - stickR);
            backRightPower = (stickY + stickX - stickR);

        }

        // If going backward, do it slower
        //if (stickY<0) stickY /= 2;

        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)));
        if (maxPower>1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower/= maxPower;
            backRightPower/= maxPower;
        }

        //these codes just set the power for everything
        ((DcMotorEx) Common.leftFrontDrive).setVelocity(frontLeftPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.leftBackDrive).setVelocity(backLeftPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.rightFrontDrive).setVelocity(frontRightPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.rightBackDrive).setVelocity(backRightPower * DRIVE_TICKS_PER_SEC_MAX);
    }
}
