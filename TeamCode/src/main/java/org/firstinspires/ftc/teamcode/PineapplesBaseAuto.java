package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;


// if the order is GPP
// shoot 2 purples, out take green
// go to next set of 3 and do PGP (set 1)
// go to next set of 3 and do PGP (set 3)
// go to middle and outake G and do PP


// if the order if PGP
// shoot PGP
// go to next set of 3 and do PGP (set 1)
// go to next set of 3 and do PGP (set 3)
// go to middle and outake G and do PP


// if the order is PPG
// shoot P outaken rest
// go to next set of 3 and do PGP (set 1)
// go to next set of 3 and do PGP (set 3)
// go to middle and outake G and do PP




@Configurable
public abstract class PineapplesBaseAuto extends OpMode {
    public enum Alliance{ BLUE, RED };

    protected abstract Alliance getAlliance();
    private static final double FIELD_SIZE_IN = 140.0; // 12 ft
    // Blue->Red mirror: y' = 144 - y, heading' = PI-heading

    protected Pose mirrorRedToBlue(Pose redPose) {
        double x = FIELD_SIZE_IN - redPose.getX();
        double y = redPose.getY();
        double h = AngleUnit.normalizeRadians(Math.PI - redPose.getHeading());
        return new Pose(x, y, h);
    }
    protected Pose[] poseArray;


    /* ================= CONSTANTS 1 ================= */
    double NORMAL_SPEED = 0.45;
    double SLOW_SPEED = 0.35;
    double INTAKE_RPM = 1300;
    long PRE_SHOOT_DELAY_MS = 0;
    long SHOOT_WAIT_MS = 0;
    long INTAKE_WAIT_MS = 300;
    long SECOND_INTAKE_EXTRA_MS = 300;
    double DRIVE_POWER_NORMAL = 1.0;
    double DRIVE_POWER_INTAKE = 0.85; // tune this


    double MIDDLE_BASE_POSITION = 0.69;
    double RIGHT_BASE_POSITION = 0.69;
    double LEFT_BASE_POSITION = 0.69;
    double shootingConst = 1450;


    long shooterStableSince = 0;

    static final int SHOOTER_TOL = 10;
    static final int SHOOTER_STABLE_MS = 250;

    /* ================= RED POSES ================= */
    private static final Pose[] RED_POSES = {
            new Pose(115, 118, Math.toRadians(35)),    // start
            new Pose(96, 109, Math.toRadians(295)), // check limelight

            new Pose(84, 84, Math.toRadians(225)),     // shoot 1
            new Pose(111, 100, Math.toRadians(-90)),
            new Pose(111, 80, Math.toRadians(-90)),
            new Pose(84, 84, Math.toRadians(225)),    // shoot 2
//            new Pose(110, 80, Math.toRadians(-90)),
//            new Pose(110, 53, Math.toRadians(-90)),
//            new Pose(86, 86, Math.toRadians(225)),   // shoot 3
            new Pose(110, 53, Math.toRadians(-90)),
            new Pose(110, 35, Math.toRadians(-90)),
            new Pose(84, 84, Math.toRadians(225)),    // shoot 3


    };


    /* ================= INTERNAL ================= */
    private PathChain[] paths;
    private Follower follower;
    private int state = 0;
    private long stateStartTime = 0;
    private boolean arrived = false;
    private int gateStep = 0;
    private boolean preShootDelayDone = false;
    private boolean firstShootPositionReached = false;
    private long shootPositionTime = 0;

    static final double M_UP = 0.785;
    static final double R_DOWN = 0.57;
    static final double L_DOWN = 0.58;
    static final double R_UP = 0.78;
    static final double L_UP = 0.78;

    private boolean advanced = false;
    private double shootingspeed = shootingConst;



    protected void rightGateDown() {
        if (getAlliance() == Alliance.RED)
            Common.radvance.setPosition(R_DOWN);
        else
            Common.ladvance.setPosition(L_DOWN);
    }

    protected void leftGateDown() {
        if (getAlliance() == Alliance.RED)
            Common.ladvance.setPosition(L_DOWN);
        else
            Common.radvance.setPosition(R_DOWN);
    }

    protected void rightGateUp() {
        if (getAlliance() == Alliance.RED)
            Common.radvance.setPosition(R_UP);
        else
            Common.ladvance.setPosition(L_UP);
    }

    protected void leftGateUp() {
        if (getAlliance() == Alliance.RED)
            Common.ladvance.setPosition(L_UP);
        else
            Common.radvance.setPosition(R_UP);
    }

    /* ================= SHOOTING LOGIC ================= */
    // private enum ShootStepType {AIM, PREPARE, CHECK, M_UP, R_DOWN, L_DOWN, DONE}


    static class ShootStep {
        final String name;
        final long durationMs; // -1 = condition-based


        ShootStep(String name, long durationMs) {
            this.name = name;
            this.durationMs = durationMs;
        }
    }


    private ShootStep[] shootingSteps = {
            new ShootStep("aim", 1500),      // limelight gated
            new ShootStep("prepare", 0),   // immediate
            new ShootStep("check", 250),  // shooter stable gated
            new ShootStep("mup", 750),
            new ShootStep("check", 750),
            new ShootStep("rdown", 700),
            new ShootStep("check", 750),
            new ShootStep("ldown", 700),
            new ShootStep("done", -1)
    };


    private int shootingcurstep = 0;
    private long stepStartTime = 0;

    private int detectedTag = -1;
    private boolean tagLocked = false;
    private int shootCycle = -1;   // 0 = first, 1 = second, 2 = third


    /* ================= INIT ================= */
    protected Limelight3A limelight; // copy this from teleop


    @Override
    public void init() {
        // existing init code...


        Common.configRobot(hardwareMap, false);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[0]);
        buildPaths();

        // gates neutral
        Common.radvance.setPosition(RIGHT_BASE_POSITION);
        Common.madvance.setPosition(MIDDLE_BASE_POSITION);
        Common.ladvance.setPosition(LEFT_BASE_POSITION);


        // --- LIMELIGHT INIT ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        Alliance alliance = getAlliance();
        poseArray = new Pose[RED_POSES.length];

        for (int i = 0; i < RED_POSES.length; i++) {
            poseArray[i] = (alliance == Alliance.RED)
                    ? RED_POSES[i]
                    : mirrorRedToBlue(RED_POSES[i]);
        }

    }


    @Override
    public void start() {
        state = 1;
        goToState(state);
    }


    /* ================= LOOP ================= */
    @Override
    public void loop() {


        telemetry.clear();


        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());


        follower.update();


        // default intake running
        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);


        DcMotorEx s1 = (DcMotorEx) Common.shoot;
        DcMotorEx s2 = (DcMotorEx) Common.shoot2;
        s1.setVelocity(shootingspeed);
        s2.setVelocity(-shootingspeed);


        if (state == 1) {   // CHECK LIMELIGHT
            handleCheckLimelight();
            return;
        }

        if (state == 2) {   // SHOOT 1
            handleShoot();
            return;
        }

        if (state == 3) {   // DRIVE TO STACK 1
            handleMovement();
            return;
        }

        if (state == 4) {   // INTAKE 1
            handleIntake();
            return;
        }

        if (state == 5) {   // SHOOT 2
            handleShoot();
            return;
        }

        if (state == 6) {   // DRIVE TO STACK 2
            handleMovement();
            return;
        }

        if (state == 7) {   // INTAKE 2
            handleIntake();
            return;
        }

        if (state == 8) {   // SHOOT 3
            handleShoot();
            return;
        }




        if (arrived && System.currentTimeMillis() - stateStartTime > 200) {
            advance();
        }
    }


    /* ================= SHOOT HANDLER ================= */
    private void handleMovement() {

        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
        }

        if (arrived && System.currentTimeMillis() - stateStartTime > 200) {
            advance();
        }
    }

    private void handleShoot() {



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
        long now = System.currentTimeMillis();


        Common.advancewheel.setPower(1);


        advanced = false;


        switch (step.name) {

            case  "aim":
                shootingspeed = shootingConst;
                double turnCmd = limelightTurnCmd();
                follower.setTeleOpDrive(0, 0, -turnCmd, true);


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
                shootingspeed = shootingConst;

                follower.setTeleOpDrive(0, 0, 0, false);
                follower.startTeleopDrive();
                //Common.madvance.setPosition(M_DOWN);
                rightGateUp();
                leftGateUp();
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

            case "mdown":
                Common.madvance.setPosition(MIDDLE_BASE_POSITION);
                if (elapsed >= step.durationMs) nextShootStep();
                break;

            case "rdown":
                rightGateDown();
                if (elapsed >= step.durationMs) nextShootStep();
                break;


            case "ldown":
                leftGateDown();
                if (elapsed >= step.durationMs) nextShootStep();
                break;


            case "increase":
                shootingspeed += 10;
                nextShootStep();
                break;


            case "decrease":
                shootingspeed -= 10;
                nextShootStep();
                break;

            case "outtake":
                shootingspeed = 500;
                if (elapsed >= step.durationMs) nextShootStep();
                break;

            case "done":
                advance();
                // hold shooter on, no auto-advance
                break;

        }


        telemetry.update();
    }


    void nextShootStep() {
        shootingcurstep++;
        stepStartTime = System.currentTimeMillis();
    }




    /* ================= INTAKE HANDLER ================= */
    private void handleIntake() {
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
        }


        if (!arrived) return;


        long wait = INTAKE_WAIT_MS;


        if (state == 6) wait += SECOND_INTAKE_EXTRA_MS;


        if (System.currentTimeMillis() - stateStartTime >= wait) advance();
    }


    /* ================= STATE MACHINE ================= */
    private void advance() {
        state++;

        if (state > paths.length - 1) {
            follower.setTeleOpDrive(0,0,0,true);
            return;
        }

        goToState(state);
    }



    private void goToState(int s) {

        arrived = false;
        shootingcurstep = 0;
        stepStartTime = 0;

        if (s == 4 || s == 7) {
            follower.setMaxPower(DRIVE_POWER_INTAKE);
        } else {
            follower.setMaxPower(DRIVE_POWER_NORMAL);
        }

        if (s != 1 && s != 2 && s != 5 && s != 8) {
            follower.followPath(paths[s], true);
        }
        if (s == 2 || s == 5 || s == 8) {
            aimPrevError = 0;
            aimPrevTime = 0;
            shootCycle++;
            shootingcurstep = 0;
            configureShootingForTagAndCycle();

        }


    }








    private void buildPaths() {
        paths = new PathChain[poseArray.length];
        for (int i = 1; i < poseArray.length; i++) {
            boolean slow = (i == 4 || i == 7);
            double speed = slow ? SLOW_SPEED : NORMAL_SPEED;
            paths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(poseArray[i - 1], poseArray[i]))
                    .setLinearHeadingInterpolation(
                            poseArray[i - 1].getHeading(),
                            poseArray[i].getHeading(),
                            speed
                    )
                    .build();
        }
    }


    // LIMELIGHT CODE:
    static double AIM_TX_OFFSET_DEG = 0.0;
    static final double AIM_TX_TOL_DEG = 0.5;


    static double AIM_KD = 0.002;   // start small
    static double AIM_KP = 0.01;


    private double aimPrevError = 0.0;
    private long aimPrevTime = 0;
    static final double AIM_MAX_TURN = 0.12;


    static boolean AIM_INVERT = false;       // flip ONLY if needed
    long aimStableSince = 0;
    static final int AIM_STABLE_MS = 120;


    private int scanForTag() {

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return -1;

        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags.isEmpty()) return -1;

        return tags.get(0).getFiducialId();
    }

    private void handleCheckLimelight() {

        follower.setTeleOpDrive(0, 0, 0, true);

        if (!tagLocked) {
            int tagId = scanForTag();
            if (tagId != -1) {
                detectedTag = tagId;
                tagLocked = true;
            }
        }


        if (stepStartTime == 0)
            stepStartTime = System.currentTimeMillis();

        if (System.currentTimeMillis() - stepStartTime > 500) {
            stepStartTime = 0;
            advance();
        }
    }



    private double limelightTurnCmd() {
        LLResult r = limelight.getLatestResult();
        if (!limelightHasValidTarget(r)) {
            aimPrevError = 0;
            aimPrevTime = 0;
            return 0.0;
        }



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




    private boolean limelightAligned() {
        LLResult r = limelight.getLatestResult();
        if (!limelightHasValidTarget(r)) return false;


        return Math.abs(r.getTx() - AIM_TX_OFFSET_DEG) <= AIM_TX_TOL_DEG;
    }






    private boolean limelightHasValidTarget(LLResult r) {
        if (r == null || !r.isValid()) return false;


        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        if (tags.isEmpty()) return false;



        // Limelight primary target = index 0
        return (tags.get(0).getFiducialId() != 23 && tags.get(0).getFiducialId() != 22 && tags.get(0).getFiducialId() != 21);
    }


    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void configureShootingForTagAndCycle() {

        if (detectedTag == 21) {

            if (shootCycle == 0) {
                // FIRST shooting position logic for tag 21
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("outtake", 1000),
                        new ShootStep("mup", 500),
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
                        new ShootStep("rdown", 500),
                        new ShootStep("check", 500),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 1) {
                // SECOND shooting position logic for tag 21
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 2) {
                // THIRD shooting position logic for tag 21
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }
        }
        else if (detectedTag == 22) {

            if (shootCycle == 0) {
                // FIRST shooting position logic for tag 22
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 1) {
                // SECOND shooting position logic for tag 22
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 2) {
                // THIRD shooting position logic for tag 22
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }

        }
        else if (detectedTag == 23) {

            if (shootCycle == 0) {
                // FIRST shooting position logic for tag 23
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 1) {
                // SECOND shooting position logic for tag 23
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 2) {
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 250),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
        }
    }

}



