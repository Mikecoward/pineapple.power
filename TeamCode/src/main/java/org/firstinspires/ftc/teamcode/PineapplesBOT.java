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

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//1
@Configurable
public abstract class PineapplesBOT extends OpMode {
    public enum Alliance { BLUE, RED }

    // ---- override in subclasses ----
    protected abstract Alliance getAlliance();

    public static final boolean DRAW_PATHS = true;
    public static final boolean DRAW_ROBOT = true;

    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    protected int numPaths = 5;
    protected Supplier<PathChain>[] pathArray;

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;

    protected Limelight3A limelight;

    // Smooth command state
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 0.05;


    // BLUE “source of truth”
    //80 118 - gate
    //29.8 - 104 135
    protected static final Pose[] poseArrayBlue = {
            new Pose(6.86, 135.11, Math.toRadians(0)), // 0 Blue Start Pose
            new Pose(72, 72, Math.toRadians(135)), // 1 Blue shoot1 Pose
            new Pose(9, 60, Math.toRadians(175)),// 2 Blue shoot2 Pose
            new Pose(120, 120, Math.toRadians(-180)),// 3 Blue Pickup Pose
            new Pose(30, 90, Math.toRadians(-180)) //
    };

    // Alliance-specific poses (computed at init)
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        start(0),
        shooting1(1),
        shooting2(2),
        PICKUP(3),
        AUTO_A_END(4);

        public final int value;

        AutoTarget(int value) {
            this.value = value;
        }
    }

    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    boolean toggle23 = false;

    boolean swapDirections = false;
    boolean debounceDirection = false;

    IMU imu;

    double DRIVE_TICKS_PER_SEC_MAX = 2800.0;



    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = poseArrayBlue[i];
        }

        follower = Constants.createFollower(hardwareMap);
        startingPose = poseArray[0]; // Blue start pose
        follower.setStartingPose(startingPose);
        follower.update();

        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathArray = new Supplier[numPaths];
        for (int i = 0; i < numPaths; i++) {
            final int index = i;
            pathArray[index] = () -> follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, poseArray[index])))
                    .setHeadingInterpolation(
                            HeadingInterpolator.linearFromPoint(
                                    follower::getHeading,
                                    poseArray[index].getHeading(),
                                    0.8
                            )
                    )
                    .build();
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        Common.radvance.setPosition(.55);
        Common.madvance.setPosition(.56);
        Common.ladvance.setPosition(.535);

    }

    public void start() {
        follower.startTeleopDrive();
    }
     double POSITION_TOLERANCE_INCHES = 5.0; // Only update if within 5 inches
     double HEADING_TOLERANCE_RAD = Math.toRadians(10); // Only update if within 10 degrees

    double intakingspeed = 700;
    double shootingspeed = 0;
    boolean apath = false;

    // position in triangle checking:
    boolean pointInTriangle(
            double px, double py,
            double ax, double ay,
            double bx, double by,
            double cx, double cy
    ) {
        double v0x = cx - ax;
        double v0y = cy - ay;
        double v1x = bx - ax;
        double v1y = by - ay;
        double v2x = px - ax;
        double v2y = py - ay;

        double dot00 = v0x * v0x + v0y * v0y;
        double dot01 = v0x * v1x + v0y * v1y;
        double dot02 = v0x * v2x + v0y * v2y;
        double dot11 = v1x * v1x + v1y * v1y;
        double dot12 = v1x * v2x + v1y * v2y;

        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    boolean inLaunchZone() {
        Pose p = follower.getPose();
        double x = p.getX();
        double y = p.getY();

        boolean inClose = pointInTriangle(
                x, y,
                0, 144,
                72, 72,
                144, 144
        );

        boolean inFar = pointInTriangle(
                x, y,
                48, 0,
                72, 24,
                96, 0
        );

        return inClose || inFar;
    }


    @Override
    public void loop() {
        follower.update();
        //updatePoseFromLL();  // now continuously updating

        telemetry.clear();

        if (!automatedDrive) {

            intakingspeed = 1200;
            shootingspeed = 50;

            if (Common.radvance.getPosition() != .55) {
                Common.radvance.setPosition(.55);
            }
            if (Common.ladvance.getPosition() != .535) {
                Common.ladvance.setPosition(.535);
            }
            if (Common.madvance.getPosition() != .56) {
                Common.madvance.setPosition(.56);
            }

            double targetY    = gamepad1.right_stick_y * Math.pow(Math.abs(gamepad1.right_stick_y), 1.2);
            double targetX    = gamepad1.right_stick_x * Math.pow(Math.abs(gamepad1.right_stick_x), 1.2);
            double targetTurn = gamepad1.left_stick_x  * Math.pow(Math.abs(gamepad1.left_stick_x),  1.5);

            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            double mult = slowMode ? slowModeMultiplier : 1.0;
            follower.setTeleOpDrive(
                    -cmdY * mult,
                    -cmdX * mult,
                    -cmdTurn * mult,
                    true // Robot centric (as you had)
            );
        }



        if (gamepad1.aWasPressed() && !automatedDrive) {
            Pose p = follower.getPose();
            int distshooting1, distshooting2;

            distshooting2 = (int) Math.round(Math.sqrt(Math.pow(p.getX() - 9, 2) + Math.pow(p.getY() - 60, 2)));
            distshooting1 = (int) Math.round(Math.sqrt(Math.pow(p.getX() - 72, 2) + Math.pow(p.getY() - 72, 2)));

            if (distshooting1 < distshooting2) {
                follower.followPath(pathArray[AutoTarget.shooting1.value].get());
                automatedDrive = true;
                currentAutoTarget = AutoTarget.shooting1;

            } else {
                follower.followPath(pathArray[AutoTarget.shooting2.value].get());
                automatedDrive = true;
                currentAutoTarget = AutoTarget.shooting2;

            }
        }

        if (!gamepad1.a && automatedDrive && (currentAutoTarget == AutoTarget.shooting1 || currentAutoTarget == AutoTarget.shooting2)) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.bWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.start.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.start;
        }

        if (!gamepad1.b && automatedDrive && currentAutoTarget == AutoTarget.start) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }
        //add in launch zone later
        if ( gamepad1.left_bumper) {
            Pose p = follower.getPose();
            int distshooting1, distshooting2;
            distshooting2 = (int) Math.round(Math.sqrt(Math.pow(p.getX() - 9, 2) + Math.pow(p.getY() - 60, 2)));
            distshooting1 = (int) Math.round(Math.sqrt(Math.pow(p.getX() - 72, 2) + Math.pow(p.getY() - 72, 2)));

            if (distshooting1 < distshooting2) {
                shootingspeed = 1325;
            } else {
                shootingspeed = 1325;
            }

            intakingspeed = 1300;
            long a = System.currentTimeMillis();

            while (((DcMotorEx) Common.intaking).getVelocity() <= .90 * intakingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() <= .80 * shootingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() >= .90 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() >= -.80 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() <= -.90 * shootingspeed ||
                    System.currentTimeMillis() - a <= 2000) {

                telemetry.clear();
                telemetry.addData("-- shooting motor velocity", ((DcMotorEx) Common.shoot).getVelocity());
                telemetry.addData("-- shooting2 motor velocity", ((DcMotorEx) Common.shoot2).getVelocity());
                telemetry.addData("-- intaking motor velocity", ((DcMotorEx) Common.intaking).getVelocity());
                telemetry.update();
                ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
                ((DcMotorEx) Common.shoot).setVelocity(.85 * shootingspeed);
                ((DcMotorEx) Common.shoot2).setVelocity(.85 * -shootingspeed);
            }

            Common.ladvance.setPosition(.71);
            a = System.currentTimeMillis();
            while (((DcMotorEx) Common.intaking).getVelocity() <= .90 * intakingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() <= .80 * shootingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() >= .90 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() >= -.80 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() <= -.90 * shootingspeed ||
                    System.currentTimeMillis() - a <= 2000) {

                telemetry.clear();
                telemetry.addData("-- shooting motor velocity", ((DcMotorEx) Common.shoot).getVelocity());
                telemetry.addData("-- shooting2 motor velocity", ((DcMotorEx) Common.shoot2).getVelocity());
                telemetry.addData("-- intaking motor velocity", ((DcMotorEx) Common.intaking).getVelocity());
                telemetry.update();
                ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
                ((DcMotorEx) Common.shoot).setVelocity(.85 * shootingspeed);
                ((DcMotorEx) Common.shoot2).setVelocity(.85 * -shootingspeed);
            }
            Common.radvance.setPosition(.71);

            a = System.currentTimeMillis();
            while (((DcMotorEx) Common.intaking).getVelocity() <= .90 * intakingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() <= .95 * shootingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() >= 1.05 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() >= -.95 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() <= -1.05 * shootingspeed ||
                    System.currentTimeMillis() - a <= 2000) {

                telemetry.clear();
                telemetry.addData("-- shooting motor velocity", ((DcMotorEx) Common.shoot).getVelocity());
                telemetry.addData("-- shooting2 motor velocity", ((DcMotorEx) Common.shoot2).getVelocity());
                telemetry.addData("-- intaking motor velocity", ((DcMotorEx) Common.intaking).getVelocity());
                telemetry.update();
                ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
                ((DcMotorEx) Common.shoot).setVelocity(shootingspeed);
                ((DcMotorEx) Common.shoot2).setVelocity(-shootingspeed);
            }
            Common.madvance.setPosition(.718);
            a = System.currentTimeMillis();
            while (((DcMotorEx) Common.intaking).getVelocity() <= .90 * intakingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() <= .95 * shootingspeed ||
                    ((DcMotorEx) Common.shoot).getVelocity() >= 1.05 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() >= -.95 * shootingspeed ||
                    ((DcMotorEx) Common.shoot2).getVelocity() <= -1.05 * shootingspeed ||
                    System.currentTimeMillis() - a <= 4000) {

                ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
                ((DcMotorEx) Common.shoot).setVelocity(shootingspeed);
                ((DcMotorEx) Common.shoot2).setVelocity(-shootingspeed);
            }
        }

        //motors
        ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
        ((DcMotorEx) Common.shoot).setVelocity(shootingspeed);
        ((DcMotorEx) Common.shoot2).setVelocity(-shootingspeed);



        Pose pose = follower.getPose();

        telemetry.addLine("---- Pose ----");
        telemetry.addData("X", "%.2f", pose.getX());
        telemetry.addData("Y", "%.2f", pose.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(pose.getHeading()));

        telemetry.addLine("---- pedropathing ----");
        telemetry.addData("automated?", automatedDrive);
        telemetry.addData("currentAutoTarget", currentAutoTarget);

        telemetry.addLine("---- shooting ----");
        telemetry.addLine("------ motors ----");

        telemetry.addData("-- shooting motor velocity", ((DcMotorEx) Common.shoot).getVelocity());
        telemetry.addData("-- shooting2 motor velocity", ((DcMotorEx) Common.shoot2).getVelocity());
        telemetry.addData("-- intaking motor velocity", ((DcMotorEx) Common.intaking).getVelocity());
        telemetry.addData("-- possible to shoot?", inLaunchZone());

        telemetry.addLine("------ servos ----");
        telemetry.addData("radvance position", Common.radvance.getPosition());
        telemetry.addData("madvance position", Common.radvance.getPosition());
        telemetry.addData("ladvance position", Common.radvance.getPosition());

        printLimelightData();
        telemetry.addData("LL connected", limelight.isConnected());
        telemetry.addData("LL running", limelight.isRunning());
        telemetry.addData("LL ms since update", limelight.getTimeSinceLastUpdate());
        LLResult r = limelight.getLatestResult();
        telemetry.addData("LL result valid", r != null && r.isValid());
        if (r != null) telemetry.addData("LL staleness(ms)", r.getStaleness());

        telemetry.update();


    }
    protected Pose getRawLimelightPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D llpose = result.getBotpose();
        if (llpose == null) return null;

        // Convert meters to inches without any offsets
        double xInches = DistanceUnit.METER.toInches(llpose.getPosition().x);
        double yInches = DistanceUnit.METER.toInches(llpose.getPosition().y);

        YawPitchRollAngles ypr = llpose.getOrientation();
        double headingRad = ypr.getYaw(AngleUnit.RADIANS); // raw yaw in radians

        return new Pose(xInches, yInches, headingRad);
    }

    protected void printLimelightData() {
        Pose llPose = getRawLimelightPose();
        if (llPose != null) {
            telemetry.addLine("=== Limelight Raw Data ===");
            telemetry.addData("X (in)", "%.2f", llPose.getX());
            telemetry.addData("Y (in)", "%.2f", llPose.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(llPose.getHeading()));
        } else {
            telemetry.addLine("Limelight: no valid data");
        }
    }


    protected Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D llpose = result.getBotpose();
        if (llpose == null) return null;

        double xMeters = llpose.getPosition().x;
        double yMeters = llpose.getPosition().y;

        double xInches = 72 + DistanceUnit.METER.toInches(yMeters);
        double yInches = 72 - DistanceUnit.METER.toInches(xMeters);

        YawPitchRollAngles ypr = llpose.getOrientation();
        double headingRad = AngleUnit.normalizeRadians(ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(90));

        return new Pose(xInches, yInches, headingRad);
    }

    protected void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();
        if (llPose != null) {
            telemetry.addLine("LL Data Valid");

            Pose currentPose = follower.getPose();

            double dx = llPose.getX() - currentPose.getX();
            double dy = llPose.getY() - currentPose.getY();
            double dheading = normalizeAngleD(Math.toDegrees(llPose.getHeading() - currentPose.getHeading()));

            if (true) {
                follower.setPose(llPose);
                telemetry.addData("Pose updated from LL", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
            } else {
                telemetry.addData("LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                        llPose.getX(), llPose.getY(), Math.toDegrees(llPose.getHeading()));
            }
        } else {
            telemetry.addLine("No LL Data");
        }
    }

    protected static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}