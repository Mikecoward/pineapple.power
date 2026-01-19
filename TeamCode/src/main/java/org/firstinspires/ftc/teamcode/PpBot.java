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


import static org.firstinspires.ftc.teamcode.Common.intaking;
import static org.firstinspires.ftc.teamcode.Common.limelight;
import static org.firstinspires.ftc.teamcode.Common.normalizeAngleD;
import static org.firstinspires.ftc.teamcode.Common.radvance;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


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
//1
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

@TeleOp(name="AS-first robot-.2", group="Robot")
@Disabled

public class PpBot extends LinearOpMode {

    public static final boolean DRAW_PATHS = true;
    public static final boolean DRAW_ROBOT = true;

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

        //1 SET STARTING POSE MANUALLY (FIELD COORDINATES)
        Pose start = new Pose(9, 135, 0);
        follower.setStartingPose(start);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(
                        new BezierLine(follower::getPose, new Pose(72, 72))
                ))
                .setHeadingInterpolation(
                        HeadingInterpolator.linearFromPoint(
                                follower::getHeading,
                                Math.toRadians(135),
                                0.1
                        )
                )
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


        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
/*
        Common.updatePinpoint();

        Common.zeroBothMotors();
        /*
        /* Run until the driver presses stop */


        Pose2D pos = Common.ppPos;




        MyInit();
        follower.startTeleopDrive(); // manual driving initially


        // Start follower in teleop mode (required before running)
        MyStart();

        final double TURN_SPEED = 0.25; // Slower speed for alignment
        final double TX_DEADBAND = 2.0; // Target is "aligned" if tx is within +/- 1.0 degrees


        Pose2D ppPos = Common.odo.getPosition();


        //Common.madvance.setPosition(.5);
        //Common.ladvance.setPosition(.5);
        //Common.radvance.setPosition(.5);

        boolean shooting = false;

        int intakingspeed = 500;
        int shootingspeed = 250;

        boolean rbumppressed = false;

        // DEBOUNCES

        double posm = .54;
        double posl = .60;
        while (opModeIsActive()) {

            ppPos = Common.odo.getPosition(); // ppPos now holds X, Y, heading
            double currentX = ppPos.getX(DistanceUnit.INCH); // current X in inches
            double currentY = ppPos.getY(DistanceUnit.INCH); // current Y in inches


            telemetry.addData("shooting motor velocity", ((DcMotorEx) Common.shoot).getVelocity());
            telemetry.addData("shooting2 motor velocity", ((DcMotorEx) Common.shoot2).getVelocity());

            telemetry.addData("intaking motor velocity", ((DcMotorEx) Common.intaking).getVelocity());
            telemetry.addData("shooting", shooting);
            telemetry.addData("intaking speed", intakingspeed);
            telemetry.addData("shooting speed", shootingspeed);
            telemetry.addData("automated drive", automatedDrive);
            telemetry.addLine("POSITION");
            telemetry.addData("position", ppPos);
            telemetry.addData("currentX", currentX);
            telemetry.addData("currentY", currentY);

            /*
            LIMELIGHT CODE (NOT USING FOR ODOMETRY DRIFT)
            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
             */

            //Common.updatePinpoint();

            telemetry.addData("angle", Common.madvance.getPosition());
            telemetry.addData("position", posm);
            if (gamepad1.xWasPressed()) {
                /*
                Common.radvance.setPosition(0.25);
                Common.ladvance.setPosition(0.25);
                Common.madvance.setPosition(0.25);
                */
                posm = .75;
                posl = .3;

            }
            if (gamepad1.yWasPressed()) {
                /*
                Common.radvance.setPosition(.5);
                Common.ladvance.setPosition(.5);
                Common.madvance.setPosition(-.75);

                 */
                posm = .63;
                posl = .5;
            }
            if (gamepad1.bWasPressed()) {
                Common.madvance.setPosition(posm);
                //Common.radvance.setPosition(pos1);
                Common.ladvance.setPosition(posl);

            }




            ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
            ((DcMotorEx) Common.shoot).setVelocity(shootingspeed);
            ((DcMotorEx) Common.shoot2).setVelocity(shootingspeed);
            if (gamepad1.left_bumper && !shooting) {
                shooting = true;
            }


            if (!shooting && !automatedDrive) {
                intakingspeed = 900;
                //noshootingspeed set
                /*
                C1ommon.radvance.setPosition(0);
                Common.ladvance.setPosition(0);
                Common.madvance.setPosition(0);

                 */

            } else if (!automatedDrive){
                intakingspeed = 0;
                shootingspeed = 1250;

                telemetry.addLine("running path");

                // Now run the path

                follower.followPath(pathChain.get());
                automatedDrive = true;
            }

            //.Stop automated following if the follower is done
            rbumppressed = gamepad1.right_bumper;
            if (automatedDrive && (rbumppressed || !follower.isBusy())) {
                //follower.startTeleopDrive();
                telemetry.addLine("STOPPED THE PATH CUH");
                if (rbumppressed) {
                    follower.startTeleopDrive();
                    automatedDrive = false;
                    shootingspeed = 250;

                } else {
                    automatedDrive = false;
                    intakingspeed = 1300;
                    while (((DcMotorEx) Common.intaking).getVelocity() <= .9 * intakingspeed) {
                        ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
                    }
                    // start the shooting because we are in the right spot
                }
                shooting = false;

                //Common.radvance.setPosition(.2);
                //Common.ladvance.setPosition(.2);
                //Common.madvance.setPosition(.2);
            } else if (!shooting && rbumppressed) {
                shootingspeed = 250;
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
            if (gamepad1.left_bumper) {
                telemetry.addLine("Pressed bumber left");
            }
            if (gamepad1.left_trigger > 0) {
                telemetry.addData("Pressed trigger left", gamepad1.left_trigger);
            }

            if (gamepad1.start) {
                Common.setInitialPosition(0, 0, 0);
            }

            follower.update();
            if (automatedDrive) {
                // print something
            } else {
                handleJoystick();
            }
            telemetry.update();
            telemetryM.update();

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

        if (Math.abs(stickX) < 0.05 &&
                Math.abs(stickY) < 0.05 &&
                Math.abs(stickR) < 0.05) {

            ((DcMotorEx) Common.leftFrontDrive).setVelocity(0);
            ((DcMotorEx) Common.leftBackDrive).setVelocity(0);
            ((DcMotorEx) Common.rightFrontDrive).setVelocity(0);
            ((DcMotorEx) Common.rightBackDrive).setVelocity(0);

            return; // IMPORTANT: stop here
        }

        //these codes just set the power for everything
        ((DcMotorEx) Common.leftFrontDrive).setVelocity(frontLeftPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.leftBackDrive).setVelocity(backLeftPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.rightFrontDrive).setVelocity(frontRightPower * DRIVE_TICKS_PER_SEC_MAX);
        ((DcMotorEx) Common.rightBackDrive).setVelocity(backRightPower * DRIVE_TICKS_PER_SEC_MAX);
    }
}