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
    protected double driveSpeedCap = .8;


    protected boolean slowMode = false;
    protected double slowModeMultiplier = 0.5;


//  > PEDROPATHING


    protected Follower follower;
    protected boolean automatedDrive;
    protected AutoTarget currentAutoTarget = AutoTarget.NONE;


    protected int numPaths = poseArrayRed.length;
    protected Supplier<PathChain>[] pathArray;


    // BLUE “source of truth”
    //80 118 - gate
    //29.8 - 104 135
    protected static final Pose[] poseArrayRed = {

            new Pose(6.86, 135.11, Math.toRadians(0)), // 0 Blue Start Pose
            new Pose(17, 17, Math.toRadians(225)),// 3 RED Pickup Pose
            new Pose( 135, 60, Math.toRadians(20)), // RED GATE POSE
            new Pose (33, 27, Math.toRadians(225)), // RED LIFTING


    };


    // ---- Fixed shooting points ----

    protected static Pose[] SHOOT_POINTS = {
            new Pose(72, 72, Math.toRadians(225)),
            new Pose(84, 84 ,Math.toRadians(225)),
            new Pose(96, 96, Math.toRadians(225)),
            new Pose(60, 84, Math.toRadians(215))
    };


    static final double[] SHOOT_SPEEDS = {
            1500, 1360, 1300
    };
    // add 1530


    protected Pose[] poseArray;


    protected enum AutoTarget {
        NONE(-1),
        PICKUP(2),
        GATE(3),
        LIFTING(4),
        shooting1(0);




        public final int value;


        AutoTarget(int value) {
            this.value = value;
        }
    }


    public static Pose startingPose;


    IMU imu;


    double DRIVE_TICKS_PER_SEC_MAX = 2800.0;


    double MIDDLE_BASE_POSITION = 0.69;
    double RIGHT_BASE_POSITION = 0.71;
    double LEFT_BASE_POSITION = 0.69; // 70


    boolean slow = false;
    double slowtimer = 500;


    int targetPosition = 0;
    double qspeed = 0;

    static double AIM_TX_OFFSET_DEG = -3.0;


    @Override
    public void init() {
        // Build alliance-specific pose array
        poseArray = new Pose[poseArrayRed.length];

        poseArray = new Pose[poseArrayRed.length];

        boolean isBlue = getAlliance() == Alliance.BLUE;

        for (int i = 0; i < poseArrayRed.length; i++) {
            Pose p = poseArrayRed[i];

            double x = p.getX();
            double y = p.getY();
            double heading = p.getHeading();

            if (isBlue) {
                x = 144.0 - x;                     // mirror across field
                heading += Math.toRadians(90);    // rotate heading
            }

            poseArray[i] = new Pose(
                    x,
                    y,
                    normalizeAngle(heading)
            );
        }


        Common.configRobot(hardwareMap, true);


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
        Common.lifting.setPower(0.45);



        if (getAlliance() == Alliance.BLUE) {
            SHOOT_POINTS = new Pose[] {
                    new Pose(72, 72, Math.toRadians(315)),
                    new Pose(60, 84, Math.toRadians(315)),
                    new Pose(48, 96, Math.toRadians(315)),
                   //new Pose(60, 84, Math.toRadians(305))
            };
            AIM_TX_OFFSET_DEG = 3.0;
        } else {
            SHOOT_POINTS = new Pose[] {
                    new Pose(72, 72, Math.toRadians(225)),
                    new Pose(84, 84 ,Math.toRadians(225)),
                    new Pose(96, 96, Math.toRadians(225)),
                    //new Pose(60, 84, Math.toRadians(215))
            };
            AIM_TX_OFFSET_DEG = -3.0;
        }
    }


    public void start() {
        follower.startTeleopDrive();
    }


    // LOOK UP TABLE
    // Distance (inches) -> Shooter speed (ticks/sec)
    /*
    static double[][] DISTANCE_SPEED_TABLE = {
            { 45, 1230 },
            { 50, 1290 },
            { 52, 1300 },
            { 53, 1320 },
            { 54, 1325 },
            { 55, 1325 },
            { 56, 1330 },
            { 58, 1340 },
            { 60, 1340 },
            { 62, 1365 },
            { 64, 1375 },
            { 66, 1380 },
            { 68, 1380 },
            { 70, 1400 },
            { 72, 1405 },
            { 74, 1405 },
            { 76, 1425 },
            { 78, 1430 },
            { 80, 1450 },
            { 82, 1455 },
            { 84, 1500 },
            { 85, 1505 },
            { 100, 1940 },


    };


     */










    double intakingspeed = 1200;
    double shootingspeed = 0;




    boolean centric = false;


    static final double DEADBAND = 0.06;


    private double deadband(double v) {
        return (Math.abs(v) < DEADBAND) ? 0.0 : v;
    }




    // LIMELIGHT CODE:
    static final double AIM_TX_TOL_DEG = 0.5;


    static double AIM_KD = 0.002;   // start small
    static double AIM_KP = 0.01;


    private double aimPrevError = 0.0;
    private long aimPrevTime = 0;
    static final double AIM_MAX_TURN = 0.12;


    static boolean AIM_INVERT = false;       // flip ONLY if needed
    long aimStableSince = 0;
    static final int AIM_STABLE_MS = 120;


    int CheckShootTime = 500;

    private double limelightTurnCmd() {
        LLResult r = limelight.getLatestResult();
        if (!limelightHasValidTarget(r)) return 0.0;


        double tx = r.getTx();   // safe now
        double error = tx - AIM_TX_OFFSET_DEG;


        if (Math.abs(error) <= AIM_TX_TOL_DEG) {
            aimPrevError = 0;
            return 0.0;
        }


        long now = System.nanoTime();
        double dt = (aimPrevTime == 0) ? 0.02 : (now - aimPrevTime) / 1e9;


        dt = Math.max(dt, 0.01); // add this
        double derivative = (error - aimPrevError) / dt;


        double turn = AIM_KP * error + AIM_KD * derivative;
        turn = clamp(turn, -AIM_MAX_TURN, AIM_MAX_TURN);


        aimPrevError = error;
        aimPrevTime = now;


        return AIM_INVERT ? -turn : turn;
    }






    static final int LIFT_UP_POS = 400;
    static final int LIFT_DOWN_POS = 0;




    // CASE LOGIC
    String curstep = "stagnant";


    // shooting logic,
    // --> shooterStableSince is how long the shooter was stable and in range for
    // --> SHOOTER_TOL is how much tolerance is both ways (20 degrees)
    // --> SHOOTER_STABLE_MS how long it has to be stable for1


    long shooterStableSince = 0;
    static final int SHOOTER_TOL = 5;
    static final int SHOOTER_STABLE_MS = 500;




    static final double M_UP = 0.785;
    static final double M_DOWN = 0.720;


    static final double R_DOWN = 0.625;
    static final double L_DOWN = 0.615;


    static final double R_UP = 0.79;
    static final double L_UP = 0.78;


    int shootingcurstep = 0;


    long stepStartTime = 0;


    void nextShootStep() {
        shootingcurstep++;
        stepStartTime = System.currentTimeMillis();


        // HARD reset aim controller
        aimPrevError = 0;
        aimPrevTime = 0;
    }




    ShootStep[] shootingSteps = {
            new ShootStep("aim", 600),        // limelight gated
            new ShootStep("prepare", 0),     // immediate
            new ShootStep("increase",2),
            new ShootStep("check", 250),
            new ShootStep("mup", 400),
            new ShootStep("decrease", 2),
            new ShootStep("check", 750),       // shooter stable gated
            new ShootStep("rdown", 500),
            new ShootStep("check", 750),       // shooter stable gated
            new ShootStep("ldown", 500),
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
    double localizedtime = 0.0;


    // ---- Lookup table tuning ----
    int selectedTableIndex = 0;
    static final double SPEED_STEP = 25;   // how much each button press changes speed


    boolean shooterEnabled = false;
    double shooterElapse = 0.0;


    @Override
    public void loop() {


        if (gamepad1.right_trigger > 0.6) {
            updatePoseFromLL();
        }

        follower.update();


        Pose p = follower.getPose();
        double curx = p.getX();
        double cury = p.getY();
        double curhead = Math.toDegrees(p.getHeading());






        Pose llpose = getRobotPoseFromCamera();


        if (llpose != null && !automatedDrive && !"shooting".equals(curstep)) {
            double distan = distanceTOGOAL(curx, cury);
            distan = Math.max(distan, 50);
            curx += (llpose.getX() - curx) * 1 / distan;
            cury += (llpose.getY() - cury) * 1 / distan;


        }



        telemetry.clear();
        telemetry.addData("trigger", gamepad1.right_trigger);


        //1






        if (!automatedDrive && !"shooting".equals(curstep)) {


            // GAMEPAD CONTROLS:
            double rawY = deadband(gamepad1.right_stick_y);
            double rawX = deadband(gamepad1.right_stick_x);
            double rawTurn = deadband(gamepad1.left_stick_x);


            double targetY    = rawY    * Math.pow(Math.abs(rawY),    1.5);
            double targetX    = rawX    * Math.pow(Math.abs(rawX),    1.5);
            double targetTurn = (rawTurn * Math.pow(Math.abs(rawTurn), 1.5)) / 1.5;


            // X/Y slew is fine if you like it
            cmdY += clamp(targetY - cmdY, -JOYSTICK_SLEW, JOYSTICK_SLEW);
            cmdX += clamp(targetX - cmdX, -JOYSTICK_SLEW, JOYSTICK_SLEW);


            // TURN: snap to zero when centered (prevents “random” yaw drift)
            if (targetTurn == 0.0) cmdTurn = 0.0;
            else cmdTurn += clamp(targetTurn - cmdTurn, -JOYSTICK_SLEW, JOYSTICK_SLEW);


            double mult = slowMode ? slowModeMultiplier : 1.0;
            mult *= driveSpeedCap; // Apply speed cap to all movements

            double driveX = cmdX;
            double driveY = cmdY;

            if (getAlliance() == Alliance.BLUE) {
                driveX = -driveX;
                driveY = -driveY;
            }


            if (targetY == 0 && targetX == 0 && targetTurn == 0) {
                follower.setTeleOpDrive( 0, 0, 0, false );
            } else {
                follower.setTeleOpDrive(-driveY * mult, -driveX * mult, -cmdTurn * mult, false);
            }
        }


        // ---------------> SHOOTING (dynamic closest point on y=x from (72,72)->(90,90))
        if (gamepad1.dpad_up && !automatedDrive) {


            shootingspeed = 1300;
            shooterEnabled = true;


            int shootIndex = closestShootIndex(follower.getPose());
            Pose target = SHOOT_POINTS[shootIndex];

            double a = distShootPos(follower.getPose().getX(), follower.getPose().getY(), target.getX(), target.getY());
            if (a > 70) {
                shootingSteps = new ShootStep[] {
                        new ShootStep("increase", 5),
                        new ShootStep("aim", 600),        // limelight gated
                        new ShootStep("prepare", 0),     // immediate
                        new ShootStep("check", 600),
                        new ShootStep("mup", 400),
                        new ShootStep("decrease", 5),
                        new ShootStep("check", 750),       // shooter stable gated
                        new ShootStep("rdown", 500),
                        new ShootStep("check", 750),       // shooter stable gated
                        new ShootStep("ldown", 500),
                        new ShootStep("done", -1)
                };
            } else {
                shootingSteps = new ShootStep[] {
                        new ShootStep("increase", 5),
                        new ShootStep("aim", 600),        // limelight gated
                        new ShootStep("prepare", 0),     // immediate
                        new ShootStep("check", 1000),
                        new ShootStep("mup", 400),
                        new ShootStep("decrease", 2),
                        new ShootStep("check", 750),       // shooter stable gated
                        new ShootStep("rdown", 500),
                        new ShootStep("check", 750),       // shooter stable gated
                        new ShootStep("ldown", 500),
                        new ShootStep("done", -1)
                };
            }


            qspeed = SHOOT_SPEEDS[shootIndex];


            PathChain shootPath = follower.pathBuilder()
                    .addPath(new Path(new BezierLine(follower::getPose, target)))
                    .setVelocityConstraint(5)
                    .setTranslationalConstraint(1.0)
                    .setHeadingConstraint(.01)


                    .setHeadingInterpolation(
                            HeadingInterpolator.linearFromPoint(
                                    follower::getHeading,
                                    target.getHeading(),
                                    0.8
                            )
                    )
                    .build();


            follower.followPath(shootPath);
            automatedDrive = true;
            currentAutoTarget = AutoTarget.shooting1; // reuse this state
        }


        if (!gamepad1.dpad_up && automatedDrive && currentAutoTarget == AutoTarget.shooting1) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }






        // ---------------> PICKUP


        if (gamepad1.dpad_down && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.PICKUP.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.PICKUP;
        }




        if (!gamepad1.dpad_down && automatedDrive && currentAutoTarget == AutoTarget.PICKUP) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }


        // ---------------> GATE


        if (gamepad1.dpad_left && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.GATE.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.GATE;
        }


        if (!gamepad1.dpad_left && automatedDrive && currentAutoTarget == AutoTarget.GATE) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }


        // ----------------> LIFTING


        if (gamepad1.dpad_right && !automatedDrive) {
            follower.followPath(pathArray[AutoTarget.LIFTING.value].get());
            automatedDrive = true;
            currentAutoTarget = AutoTarget.LIFTING;
        }


        if (!gamepad1.dpad_right && automatedDrive && currentAutoTarget == AutoTarget.LIFTING) {
            follower.startTeleopDrive();
            automatedDrive = false;
            currentAutoTarget = AutoTarget.NONE;
        }




        // _----------------- SHOOTING LOGIC --------------------------_


        if (!gamepad1.right_bumper) {
            curstep = "stagnant";
        } else {
            curstep = "shooting";
        }


        //add in launch zone later
        if ("shooting".equals(curstep)) {


            shooterEnabled = true;
            int shootIndex = closestShootIndex(follower.getPose());
            qspeed = SHOOT_SPEEDS[shootIndex];






            follower.setTeleOpDrive(0, 0, 0, true);


            if (stepStartTime == 0) {
                stepStartTime = System.currentTimeMillis();
            }


            telemetry.addLine("SHOOTINGNNGNGNG");


            shootingcurstep = Math.min(shootingcurstep, shootingSteps.length - 1);
            ShootStep step = shootingSteps[shootingcurstep];


            long elapsed = System.currentTimeMillis() - stepStartTime;


            double shooterVel1 = ((DcMotorEx) Common.shoot).getVelocity();
            double shooterVel2 = ((DcMotorEx) Common.shoot2).getVelocity();
            double now = System.currentTimeMillis();


            //double distance = distanceTOGOAL(curx, cury);




            Common.advancewheel.setPower(1);


            advanced = false;


            switch (step.name) {


                case  "aim":
                    double turnCmd = limelightTurnCmd();
                    follower.setTeleOpDrive(0, 0, -turnCmd, true);


                    shootingspeed = qspeed;
                    intakingspeed = 1300;


                    if (limelightAligned()) {
                        if (aimStableSince == 0)
                            aimStableSince = System.currentTimeMillis();


                        if (System.currentTimeMillis() - aimStableSince >= AIM_STABLE_MS) {
                            follower.setTeleOpDrive(0, 0, 0, true);
                            nextShootStep();
                            aimStableSince = 0;
                        }
                    } else {
                        aimStableSince = 0;
                    }


                    if (elapsed >= step.durationMs) nextShootStep();


                    break;






                case "prepare":


                    follower.setTeleOpDrive(0, 0, 0, false);
                    follower.startTeleopDrive();
                    if (shootingspeed == 1300) {
                        updatePoseFromLL();
                    }
                    shootingspeed = qspeed;
                    //Common.madvance.setPosition(M_DOWN);
                    Common.radvance.setPosition(R_UP);
                    Common.ladvance.setPosition(L_UP);
                    //Common.madvance.setPosition(M_DOWN);
                    nextShootStep();
                    break;


                case "check":
                    follower.setTeleOpDrive(0, 0, 0, true);


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
                    shootingspeed += 10 * step.durationMs;
                    nextShootStep();
                    break;


                case "decrease":
                    shootingspeed -= 10 * step.durationMs;
                    nextShootStep();
                    break;


                case "done":
                    shooterEnabled = false;
                    // hold shooter on, no auto-advance
                    break;
            }








        } else if ("stagnant".equals(curstep)) {
            if (gamepad1.left_bumper) {
                Common.radvance.setPosition(.79);
                Common.madvance.setPosition(.61);
                Common.ladvance.setPosition(.79);


                intakingspeed = 1300;
            } else {
                // || (System.currentTimeMillis() - slowtimer > 0 && slow)
                if (gamepad1.left_trigger > 0.8) {
                    intakingspeed = -500;
                } else {
                    intakingspeed = 1300;
                }


                Common.radvance.setPosition(RIGHT_BASE_POSITION);
                Common.madvance.setPosition(MIDDLE_BASE_POSITION);
                Common.ladvance.setPosition(LEFT_BASE_POSITION);


            }


            shootingcurstep = 0;
            stepStartTime = 0;
            shootingspeed = 0;
            shooterStableSince = 0;
            aimPrevError = 0;
            aimPrevTime = 0;


            Common.advancewheel.setPower(-1);


        }


        /*
        if (((DcMotorEx) Common.shoot).getVelocity() < 250) {
            slow = true;
            slowtimer = System.currentTimeMillis();
        } else {
            slow = false;
        }


         */






        // Lift control on buttons
        if (gamepad1.y) {
            Common.lifting.setTargetPosition(LIFT_UP_POS);
        }


        if (gamepad1.a) {
            Common.lifting.setTargetPosition(LIFT_DOWN_POS);
        }








        if (Common.lifting.getCurrentPosition() >= 350) {
            intakingspeed = 0;
            shootingspeed = 0;
            Common.advancewheel.setPower(0);
        }


        if (shooterEnabled) {
            int shootIndex = closestShootIndex(follower.getPose());
            shootingspeed = SHOOT_SPEEDS[shootIndex];


        } else {
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
        telemetry.addData("head ", curhead);
        telemetry.addData("Distance to goal", "%.0f", distanceTOGOAL(curx, cury));




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


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                telemetry.addData("TAG ID", tag.getFiducialId());
            }
        }


        telemetry.update();


    }


    private int closestShootIndex(Pose fromPose) {
        int bestIndex = 0;
        double smallestDist = Double.MAX_VALUE;


        for (int i = 0; i < SHOOT_POINTS.length; i++) {
            Pose p = SHOOT_POINTS[i];


            double dx = fromPose.getX() - p.getX();
            double dy = fromPose.getY() - p.getY();
            double dist = dx * dx + dy * dy;


            if (dist < smallestDist) {
                smallestDist = dist;
                bestIndex = i;
            }
        }


        return bestIndex;
    }




    protected Pose getRobotPoseFromCamera() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;


        Pose3D llpose = result.getBotpose();
        if (llpose == null) return null;


        if (!limelightHasValidTarget(result)) return null;




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
        if (!limelightHasValidTarget(r)) return false;


        return Math.abs(r.getTx() - AIM_TX_OFFSET_DEG) <= AIM_TX_TOL_DEG;
    }

    public double distShootPos(double x, double y, double x1, double y1){
        return Math.sqrt(Math.pow(y1 - y, 2) + Math.pow(x1 - x, 2));
    }





    private boolean limelightHasValidTarget(LLResult r) {
        if (r == null || !r.isValid()) return false;


        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags.isEmpty()) return false;


        // Limelight primary target = index 0
        if (getAlliance() == Alliance.BLUE) {
            return tags.get(0).getFiducialId() != 23 && tags.get(0).getFiducialId() != 22 && tags.get(0).getFiducialId() != 21 && tags.get(0).getFiducialId() != 24;
        } else {
            return tags.get(0).getFiducialId() != 23 && tags.get(0).getFiducialId() != 22 && tags.get(0).getFiducialId() != 21 && tags.get(0).getFiducialId() != 20;
        }
    }



    protected double normalizeAngle(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
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



