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

    //1 ---- override in subclasses ----
    protected abstract Alliance getAlliance();

    public static final boolean DRAW_PATHS = true;
    public static final boolean DRAW_ROBOT = true;

    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    protected int numPaths = 7;
    protected Supplier<PathChain>[] pathArray;

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;

    protected Limelight3A limelight;

    // Smooth command state
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 0.15;


    // BLUE “source of truth”
    //80 118 - gate
    //29.8 - 104 135
    protected static final Pose[] poseArrayBlue = {
            new Pose(6.86, 135.11, Math.toRadians(0)), // 0 Blue Start Pose
            new Pose(92, 92, Math.toRadians(225)), // 1 Blue shoot1 Pose
            new Pose(9, 60, Math.toRadians(175)),// 2 Blue shoot2 Pose

            new Pose(0, 144, Math.toRadians(180)),// 3 Blue Pickup Pose
            new Pose(7, 135, Math.toRadians(180)), // 4 BLue reset ODOM Pose
            new Pose( 70, 131, Math.toRadians(0)), // GATE POSE
            new Pose (25, 110, Math.toRadians(225)) // LIFTING
    };

    //1 Alliance-specific poses (computed at init)
    protected Pose[] poseArray;

    protected enum AutoTarget {
        NONE(-1),
        start(0),
        shooting1(1),
        shooting2(2),
        PICKUP(3),
        ResetODOM(4),
        GATE(5),
        LIFTING(6);

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


    int targetPosition = 0;
    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = poseArrayBlue[i];
        }

        Common.configRobot(hardwareMap, false);

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

        targetPosition = 0;
        Common.lifting.setTargetPosition(targetPosition);
        Common.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Common.lifting.setPower(0.25);
    }

    public void start() {
        follower.startTeleopDrive();
    }
     double POSITION_TOLERANCE_INCHES = 5.0; // Only update if within 5 inches
     double HEADING_TOLERANCE_RAD = Math.toRadians(10); // Only update if within 10 degrees

    double intakingspeed = 700;
    double shootingspeed = 0;

    static final double TX_TOL = 1.0;                 // degrees
    static final double DEG_TO_RAD = Math.PI / 180.0;
    static final double TURN_GAIN = 0.6;              // smaller = slower

    private void waitForShooter(
            double shooterTarget,
            long waitMS,
            boolean useTelemetry
    ) {
        final double INTAKE_TARGET = 1300;
        final double TOL_LOW  = 0.97;
        final double TOL_HIGH = 1.03;

        DcMotorEx intake = (DcMotorEx) Common.intaking;
        DcMotorEx shoot  = (DcMotorEx) Common.shoot;
        DcMotorEx shoot2 = (DcMotorEx) Common.shoot2;

        long start = System.currentTimeMillis();
        long redo = 0;
        boolean good = false;

        while (true) {
            double vI  = intake.getVelocity();
            double vS  = shoot.getVelocity();
            double vS2 = shoot2.getVelocity();

            boolean intakeOk = vI >= 0.90 * INTAKE_TARGET;

            boolean shooterOk =
                    vS  >= TOL_LOW  * shooterTarget &&
                            vS  <= TOL_HIGH * shooterTarget &&
                            vS2 <= -TOL_LOW  * shooterTarget &&
                            vS2 >= -TOL_HIGH * shooterTarget;

            if (good && !shooterOk) {
                good = false;
            }
            if (shooterOk && !good) {
                good  = true;
                redo = System.currentTimeMillis();
            }
            if (good) {
                if ((intakeOk && shooterOk && System.currentTimeMillis() - redo >= waitMS) ||
                        System.currentTimeMillis() - start >= 2000) {
                    break;
                }
            }

            intake.setVelocity(INTAKE_TARGET);
            shoot.setVelocity(shooterTarget);
            shoot2.setVelocity(-1 * shooterTarget);

            if (useTelemetry) {
                telemetry.addData("shoot", vS);
                telemetry.addData("shoot2", vS2);
                telemetry.addData("intake", vI);
                telemetry.update();
            }
        }
    }


    boolean centric = false;

    // LIMELIGHT CODE:
    // --- Limelight aim (hold right bumper) ---
    static final double AIM_TX_TOL_DEG = 1.0;     // stop when |tx| <= this
    static final double AIM_KP = 0.02;            // scalar: turnPower = KP * tx
    static final double AIM_MAX_TURN = 0.18;      // keep it slow
    static final double AIM_MIN_TURN = 0.04;      // overcome stiction; set 0 if too jumpy
    static final double AIM_TIMEOUT_S = 0.8;      // safety timeout per hold

    static boolean AIM_INVERT = true;            // flip if it turns the wrong way

    boolean aiming = false;
    long aimStartMs = 0;

    private double limelightTurnCmd() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;

        double tx = r.getTx(); // degrees
        if (Math.abs(tx) <= AIM_TX_TOL_DEG) return 0.0;

        double turn = AIM_KP * tx;

        // clamp slow
        turn = clamp(turn, -AIM_MAX_TURN, AIM_MAX_TURN);

        // minimum turn to actually move (optional)
        if (Math.abs(turn) < AIM_MIN_TURN) turn = Math.signum(turn) * AIM_MIN_TURN;

        if (AIM_INVERT) turn = -turn;
        return turn;


    }

    static final int LIFT_UP_POS = 400;
    static final int LIFT_DOWN_POS = 0;



    // CASE LOGIC
    String[] steps = {"preparing", "shootaim", "end"};
    int curstep = 3;
    @Override
    public void loop() {
        follower.update();

        telemetry.clear();
        //1

        if (!automatedDrive) {

            // GAMEPAD CONTROLS:
            double joyY = gamepad1.right_stick_y;
            double joyX = gamepad1.right_stick_x;
            double joyTurn = gamepad1.left_stick_x;

            double turnCmd;

            // default: driver turn
            turnCmd = -gamepad1.left_stick_x;

            follower.setTeleOpDrive(
                    -gamepad1.right_stick_y,
                    -gamepad1.right_stick_x,
                    turnCmd,
                    false // robot centric
            );

        }

        // CLOSEST SHOOTER
        if (gamepad1.aWasPressed() && !automatedDrive) {
            Pose p = follower.getPose();
            int distshooting1, distshooting2;

            distshooting2 = (int) Math.round(Math.sqrt(Math.pow(p.getX() - 9, 2) + Math.pow(p.getY() - 60, 2)));
            distshooting2 = 100000;
            // FOR NOW ONLY GO FOR DISTSHOOTING1
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
            follower.followPath(pathArray[AutoTarget.PICKUP.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.PICKUP;
        }

        if (!gamepad1.b && automatedDrive && currentAutoTarget == AutoTarget.PICKUP) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.xWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.GATE.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.GATE;
        }

        if (!gamepad1.x && automatedDrive && currentAutoTarget == AutoTarget.GATE) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.yWasPressed() && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.LIFTING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.LIFTING;
        }

        if (!gamepad1.y && automatedDrive && currentAutoTarget == AutoTarget.LIFTING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }

        if (gamepad1.right_trigger > .7) {
            follower.setPose(poseArray[4]);
        }

        // _----------------- SHOOTING LOGIC --------------------------_

        if (!gamepad1.left_bumper) {
            curstep = 3;
        } else if (curstep == 3) {
            curstep = 1;
        }






        //add in launch zone later
        if (curstep == 1) {

            double turnCmd = 0;
            if (gamepad1.right_bumper) {
                LLResult r = limelight.getLatestResult();
                if (r != null && r.isValid() && Math.abs(r.getTx()) > AIM_TX_TOL_DEG) {
                    turnCmd = limelightTurnCmd();
                }
            }

            follower.setTeleOpDrive(
                    0,
                    0,
                    turnCmd,
                    false // robot centric
            );

            shootingspeed = 1000;
            // check tolerances and if shooting speed is up to par
        } else if (curstep == 2) {
            if (Common.madvance.getPosition() != .725) {
                Common.madvance.setPosition(.725);
            }
        } else if (curstep == 3) {
            intakingspeed = 500;
            shootingspeed = 0;

            if (Common.radvance.getPosition() != .55) {
                Common.radvance.setPosition(.55);
            }
            if (Common.madvance.getPosition() != .56) {
                Common.madvance.setPosition(.56);
            }
            if (Common.ladvance.getPosition() != .535) {
                Common.ladvance.setPosition(.535);
            }

        }


        if (gamepad1.dpad_up) {
            Common.lifting.setTargetPosition(LIFT_UP_POS);

        }

        if (gamepad1.dpad_down) {
            Common.lifting.setTargetPosition(LIFT_DOWN_POS);
        }

        if (Common.lifting.getCurrentPosition() >= 350) {
            intakingspeed = 0;
            shootingspeed = 0;
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

        telemetry.addData("LF vel", ((DcMotorEx) Common.leftFrontDrive).getVelocity());
        telemetry.addData("LB vel", ((DcMotorEx) Common.leftBackDrive).getVelocity());
        telemetry.addData("RF vel", ((DcMotorEx) Common.rightFrontDrive).getVelocity());
        telemetry.addData("RB vel", ((DcMotorEx) Common.rightBackDrive).getVelocity());
        telemetry.addData("lifting", Common.lifting.getCurrentPosition());

        telemetry.addLine("------ servos ----");
        telemetry.addData("radvance position", Common.radvance.getPosition());
        telemetry.addData("madvance position", Common.madvance.getPosition());
        telemetry.addData("ladvance position", Common.ladvance.getPosition());

        LLResult rr = limelight.getLatestResult();
        telemetry.addLine("---- limelight ----");
        telemetry.addData("valid", rr != null && rr.isValid());
        if (rr != null && rr.isValid()) telemetry.addData("tx(deg)", rr.getTx());
        telemetry.addData("aiming", aiming);


        telemetry.update();


    }

    protected static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}