package org.firstinspires.ftc.teamcode;

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




/* Control Map

Gamepad A:
Right Stick X: move left/right
Right Stick Y: move forward/back
Left Stick X:  turn robot left/right
Left Stick Y:
Press Right/Left Stick in: Slow Down Robot

Dpad L/R:      arm in/out
Dpad Up/Down:  lift up/down
A button: Intake down
 B button:      intake up
X button:      swap joystick directions
Y button:      outtake dispense
L button:      ****Intake grab a block
L trigger:     ***raise lift, dump, back to neutral, down
R button:      ***Intake dump into bucket
R trigger:
Logo (Logitech Button): move to basket & dump, and move back to field and redeploy
back button:
start button:  Set IMU back to 0.

Gamepad 2:
Right Stick X:
Right Stick Y:
Left Stick X:
Left Stick Y:
Dpad L:
Dpad R: Move lift down a bit to lock specimen in
Dpad Up: Specimen Hanging Height, move to bars
Dpad Down: Wall Height.  move to specimen zone

A button:
B button:  sweep field
X button:  move to basket & dump, and move back to field and redeploy
Y button:  dump bucket
L button: Reverse the intake
L trigger: Wall Height
R button: Power to the intake
R trigger: Specimen Hanging Height
Logo (Logitech Button):  Specimen Mode or Sample Mode
back button:
start button:

*/

/* Needed controls:
-> Move lift all the way down
-> Take lift to hanging height
-> Drop lift to hang specimen
-> Specimen Mode (don't pick up yellow.  Don't auto drop in bucket)
-> Button to drop sample off the front for specimen mode
*/


@TeleOp(name="AS-first robot-3", group="Robot")
//@Disabled

public class PpBot extends LinearOpMode {
    double counter = 0;
    boolean isydone = false;
    boolean isnewydone = false;
    double time_in = 0;
    double starting_time = 0;
    boolean colorf_good;
    boolean colorb_good;

    boolean trial1 = false;
    boolean toggle28 = false;

    double deployDownPos = .765;
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    boolean toggledispense = true;
    double direction_shifter_1 = 1;

    boolean toggle23 = false;
    boolean toggle24 = false;
    boolean toggle25 = false;
    double counter24 = 0;
    double counter25 = 0;
    double counter23 = 0;
    double counter26 = 0;

    boolean swapDirections = false;
    boolean debounceDirection = false;

    boolean specimenMode = false;
    boolean debounceSpecimen = false;
    IMU imu;


    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */

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
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

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

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();
/*
        Common.updatePinpoint();

        Common.zeroBothMotors();
        /*
        /* Run until the driver presses stop */
        while (opModeIsActive()){
            Common.updatePinpoint();   // <-- Step 1, update Pinpoint
            telemetry.update();        // push telemetry to DS screen

            /*telemetry.addData("rangeL", String.format("%.01f mm", Common.sensorDistanceL.getDistance(DistanceUnit.MM)));
            telemetry.addData("rangeR", String.format("%.01f mm", Common.sensorDistanceR.getDistance(DistanceUnit.MM)));
            */
            if (gamepad2.start){
                Common.zeroBothMotors();
            }

            // Swap Specimen/Sample mode direction when Logitech button pressed
            if (gamepad2.guide) {
                if (!debounceSpecimen) {
                    specimenMode = !specimenMode;
                    debounceSpecimen = true;
                    if (specimenMode) {
                        Common.topLED_red.on();
                    } else {
                        Common.topLED_red.off();
                    }
                }
            } else {
                debounceSpecimen = false;
            }

            if (swapDirections){
                telemetry.addLine("direction swapped.");
            } else {
                //telemetry.addLine("normal direction.");
            }
            if (specimenMode){
                telemetry.addLine("Specimen Mode.");
            } else {
                //telemetry.addLine("Sample Mode.");
            }

            /*
            if (Common.redAlliance) {
                telemetry.addLine("Red Alliance");
            } else {
                telemetry.addLine("Blue Alliance");
            }
             */



            if (gamepad1.start) {
                Common.setInitialPosition(16.5, -63.5, 0);  // Right edge of robot aligned with second tile seam from right
            }


            // Swap Joystick direction when Logitech button pressed
            if (gamepad1.x) {
                if (!debounceDirection) {
                    swapDirections = !swapDirections;
                    debounceDirection = true;
                }
            } else {
                debounceDirection = false;
            }
            handleJoystick();
            telemetry.update();
        }
    }

    private void handleJoystick() {
    // Handle Joysticks



    double stickY = -gamepad1.right_stick_y;
    double stickX = gamepad1.right_stick_x;
    double stickR = gamepad1.left_stick_x;

    // Reverse Sticks

    if (swapDirections) {
        stickY = -stickY;
        stickX = -stickX;
    }
    double minfactor = 0.15;
    double powerfactor = .20;
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
    ((DcMotorEx) Common.leftFrontDrive).setPower(frontLeftPower);
    ((DcMotorEx) Common.leftBackDrive).setPower(backLeftPower);
    ((DcMotorEx) Common.rightFrontDrive).setPower(frontRightPower);
    ((DcMotorEx) Common.rightBackDrive).setPower(backRightPower);
}

}
