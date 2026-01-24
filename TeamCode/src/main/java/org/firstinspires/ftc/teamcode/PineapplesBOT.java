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

@Configurable
public abstract class PineapplesBOT extends OpMode {
    public enum Alliance { BLUE, RED }

    //1 ---- override in subclasses ----
    protected abstract Alliance getAlliance();

    protected Limelight3A limelight;

    // > DRIVING
    protected double cmdX = 0.0;
    protected double cmdY = 0.0;
    protected double cmdTurn = 0.0;
    protected static final double JOYSTICK_SLEW = 1;
    protected double driveSpeedCap = 0.5;

    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;

//  > PEDROPATHING

    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;

    protected int numPaths = 7;
    protected Supplier<PathChain>[] pathArray;

    // BLUE “source of truth”
    //80 118 - gate
    //29.8 - 104 135
    protected static final Pose[] poseArrayBlue = {
            new Pose(6.86, 135.11, Math.toRadians(0)), // 0 Blue Start Pose
            new Pose(72, 72, Math.toRadians(225)), // 1 Blue shoot1 Pose

            new Pose(9, 60, Math.toRadians(175)),// 2 Blue shoot2 Pose

            new Pose(0, 144, Math.toRadians(180)),// 3 Blue Pickup Pose
            new Pose(7, 135, Math.toRadians(180)), // 4 BLue reset ODOM Pose
            new Pose( 70, 131, Math.toRadians(0)), // GATE POSE
            new Pose (25, 110, Math.toRadians(225)) // LIFTING
    };

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

    IMU imu;

    double DRIVE_TICKS_PER_SEC_MAX = 2800.0;

    double MIDDLE_BASE_POSITION = 0.57;
    double RIGHT_BASE_POSITION = 0.74;
    double LEFT_BASE_POSITION = 0.74; // 70


    int targetPosition = 0;
    double qspeed = 0;

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


        Common.radvance.setPosition(RIGHT_BASE_POSITION);
        Common.madvance.setPosition(MIDDLE_BASE_POSITION);
        Common.ladvance.setPosition(LEFT_BASE_POSITION);

        targetPosition = 0;
        Common.lifting.setTargetPosition(targetPosition);
        Common.lifting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Common.lifting.setPower(0.25);


    }

    public void start() {
        follower.startTeleopDrive();
    }


    double intakingspeed = 1200;
    double shootingspeed = 0;


    boolean centric = false;


    // LIMELIGHT CODE:
    static final double AIM_TX_TOL_DEG = 2.0;     // stop when |tx| <= this
    static final double AIM_KP = 0.002;            // scalar: turnPower = KP * tx
    static final double AIM_MAX_TURN = 0.18;      // keep it slow
    static final double AIM_MIN_TURN = 0.04;      // overcome stiction; set 0 if too jumpy
    static boolean AIM_INVERT = true;            // flip if it turns the wrong way

    private double limelightTurnCmd() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;

        double tx = r.getTx(); // degrees
        double error = tx;

        if (Math.abs(error) < AIM_TX_TOL_DEG) {
            error = 0; // only kill *small* noise
            return 0;
        }

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
    String curstep = "stagnant";

    // shooting logic,
    // --> shooterStableSince is how long the shooter was stable and in range for
    // --> SHOOTER_TOL is how much tolerance is both ways (20 degrees)
    // --> SHOOTER_STABLE_MS how long it has to be stable for
    long shooterStableSince = 0;
    static final int SHOOTER_TOL = 10;
    static final int SHOOTER_STABLE_MS = 250;


    static final double M_UP = 0.715;
    static final double M_DOWN = 0.62;

    static final double R_DOWN = 0.60;
    static final double L_DOWN = 0.60;

    static final double R_UP = 0.77;
    static final double L_UP = 0.77;

    int shootingcurstep = 0;

    long stepStartTime = 0;

    void nextShootStep() {
        shootingcurstep++;
        stepStartTime = System.currentTimeMillis();
    }


    ShootStep[] shootingSteps = {
            new ShootStep("aim", -1),        // limelight gated
            new ShootStep("prepare", 0),     // immediate
            new ShootStep("check", 1000),       // shooter stable gated
            new ShootStep("mup", 500),
            new ShootStep("check", 250),       // shooter stable gated
            new ShootStep("rdown", 350),
            new ShootStep("check", 250),       // shooter stable gated
            new ShootStep("ldown", 350),
            new ShootStep("done", -1)
    };


    static class ShootStep {
        final String name;
        final long durationMs; // -1 = condition-based

        ShootStep(String name, long durationMs) {
            this.name = name;
            this.durationMs = durationMs;
        }
    }
    boolean advanced = false;

    @Override
    public void loop() {
        follower.update();
        if (gamepad1.left_trigger > 0.8) {
            updatePoseFromLL();
        }
        telemetry.clear();
        //1

        Pose p = follower.getPose();
        double curx = p.getX();
        double cury = p.getY();

        if (!automatedDrive && !"shooting".equals(curstep)) {

            // GAMEPAD CONTROLS:
            double targetY    = gamepad1.right_stick_y * Math.pow(Math.abs(gamepad1.right_stick_y), 1.5);
            double targetX    = gamepad1.right_stick_x * Math.pow(Math.abs(gamepad1.right_stick_x), 1.5);
            double targetTurn = gamepad1.left_stick_x  * Math.pow(Math.abs(gamepad1.left_stick_x),  1.5) / 1.5;

            cmdY    += clamp(targetY    - cmdY,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX    += clamp(targetX    - cmdX,    -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);

            double mult = slowMode ? slowModeMultiplier : 1.0;
            mult *= driveSpeedCap; // Apply speed cap to all movements

            follower.setTeleOpDrive(
                    -cmdY * mult,
                    -cmdX * mult,
                    -cmdTurn * mult,
                    false
            );

        }

        // ---------------> SHOOTING

        if (gamepad1.aWasPressed() && !automatedDrive) {
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

        // ---------------> PICKUP

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

        // ---------------> GATE

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

        // ----------------> LIFTING

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
            curstep = "stagnant";
        } else {
            curstep = "shooting";
        }

        //add in launch zone later
        if ("shooting".equals(curstep)) {

            if ("shooting".equals(curstep) && stepStartTime == 0) {
                stepStartTime = System.currentTimeMillis();
            }

            telemetry.addLine("SHOOTINGNNGNGNG");

            shootingcurstep = Math.min(shootingcurstep, shootingSteps.length - 1);
            ShootStep step = shootingSteps[shootingcurstep];

            long elapsed = System.currentTimeMillis() - stepStartTime;

            double shooterVel1 = ((DcMotorEx) Common.shoot).getVelocity();
            double shooterVel2 = ((DcMotorEx) Common.shoot2).getVelocity();
            double now = System.currentTimeMillis();


            advanced = false;

            switch (step.name) {

                case "aim":
                    double turnCmd = limelightTurnCmd();
                    follower.setTeleOpDrive(0, 0, turnCmd, false);
                    shootingspeed = 1000;
                    intakingspeed = 1300;

                    if (limelightAligned()) {
                        follower.setTeleOpDrive(0, 0, 0, false);
                        qspeed = 7.69 * distanceTOGOAL(curx, cury) + 838;
                        nextShootStep();
                    }
                    break;

                case "prepare":
                    shootingspeed = qspeed;
                    Common.madvance.setPosition(M_DOWN);
                    Common.radvance.setPosition(R_UP);
                    Common.ladvance.setPosition(L_UP);
                    nextShootStep();
                    break;

                case "check":
                    boolean inRange =
                            Math.abs(shooterVel1 - shootingspeed) <= SHOOTER_TOL &&
                                    Math.abs(shooterVel2 + shootingspeed) <= SHOOTER_TOL;

                    if (inRange) {
                        if (shooterStableSince == 0) shooterStableSince = System.currentTimeMillis();
                        if (System.currentTimeMillis() - shooterStableSince >= SHOOTER_STABLE_MS) {
                            nextShootStep();
                            advanced = true;
                        }
                    } else {
                        shooterStableSince = 0;

                    }
                    if (!advanced && elapsed >= step.durationMs) {
                        nextShootStep();
                    }
                    break;

                case "mup":
                    Common.madvance.setPosition(M_UP);
                    if (elapsed >= step.durationMs) nextShootStep();
                    break;

                case "rdown":
                    Common.radvance.setPosition(R_DOWN);
                    if (elapsed >= step.durationMs) nextShootStep();
                    break;

                case "ldown":
                    Common.ladvance.setPosition(L_DOWN);
                    if (elapsed >= step.durationMs) nextShootStep();
                    break;

                case "increase":
                    shootingspeed += 30;
                    nextShootStep();
                    break;

                case "decrease":
                    shootingspeed -= 30;
                    nextShootStep();
                    break;

                case "done":
                    // hold shooter on, no auto-advance
                    break;
            }




        } else if ("stagnant".equals(curstep)) {
            intakingspeed = 500;

            shootingcurstep = 0;
            stepStartTime = 0;
            shootingspeed = 0;
            shooterStableSince = 0;

            Common.radvance.setPosition(RIGHT_BASE_POSITION);
            Common.madvance.setPosition(MIDDLE_BASE_POSITION);
            Common.ladvance.setPosition(LEFT_BASE_POSITION);

        }


        if (gamepad1.dpad_up) {
            Common.lifting.setTargetPosition(LIFT_UP_POS);

        }

        if (gamepad1.dpad_down) {
            Common.lifting.setTargetPosition(LIFT_DOWN_POS);
        }

        // Change shooting steps based on location

        // Going to be variable later so for now we'll keep this constant but left
        /*
        shootingsteps = new String[] {
                "prepare",
                "check",
                "mup",
                //"mdown
                //"check",
                "rdown",
                //"mup",
                //"mdown",
                //"check",
                "ldown",
                //"check",
                //"mup",
                "done"
        };
        */


        if (Common.lifting.getCurrentPosition() >= 350) {
            // Manual Override of speed
            intakingspeed = 0;
            shootingspeed = 0;
        }

        //motors
        ((DcMotorEx) Common.intaking).setVelocity(intakingspeed);
        ((DcMotorEx) Common.shoot).setVelocity(shootingspeed);
        ((DcMotorEx) Common.shoot2).setVelocity(-shootingspeed);




        // --- CORE STATE ---
        telemetry.addData("Mode", curstep);
        telemetry.addData("ShootStep",
                shootingSteps[Math.min(shootingcurstep, shootingSteps.length - 1)].name);

        // --- POSE (only what matters) ---
        telemetry.addData("Pose (x,y)", "%.1f , %.1f", curx, cury);

        // --- SHOOTER (critical) ---
        telemetry.addData("Shooter v1", "%.0f", ((DcMotorEx) Common.shoot).getVelocity());
        telemetry.addData("Shooter v2", "%.0f", ((DcMotorEx) Common.shoot2).getVelocity());
        telemetry.addData("Target v", "%.0f", qspeed);

        // --- LIMELIGHT (only alignment info) ---
        LLResult rr = limelight.getLatestResult();
        telemetry.addData("LL valid", rr != null && rr.isValid());
        if (rr != null && rr.isValid()) {
            telemetry.addData("LL tx", "%.2f", rr.getTx());
        }

        telemetry.update();

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
        double headingRad = AngleUnit.normalizeRadians(ypr.getYaw(AngleUnit.RADIANS) - Math.toRadians(270));

        return new Pose(xInches, yInches, headingRad);
    }

    protected void updatePoseFromLL() {
        Pose llPose = getRobotPoseFromCamera();
        if (llPose != null) {
            follower.setPose(llPose);
            telemetry.addLine("LL Pose Applied");
            telemetry.addData("LL X/Y/H", "%4.2f, %4.2f, %4.1f°",
                    llPose.getX(),
                    llPose.getY(),
                    Math.toDegrees(llPose.getHeading()));

        } else {
            telemetry.addLine("No LL Data");
        }
    }

    protected static double distanceTOGOAL(double x, double y) {
        return Math.sqrt(Math.pow(128 - x, 2) + Math.pow(128 - y, 2));
    }

    private boolean limelightAligned() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid() && Math.abs(r.getTx()) <= AIM_TX_TOL_DEG;
    }

    private double expo(double input, double exponent) {
        return Math.copySign(Math.pow(Math.abs(input), exponent), input);
    }

    protected static void sleep(int ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException e) { Thread.currentThread().interrupt(); }
    }
    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}