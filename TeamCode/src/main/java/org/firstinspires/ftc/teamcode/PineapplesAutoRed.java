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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.List;


@Configurable
@Autonomous(name = "NM - Pineapple Auto RED SHOOT", group = "Autonomous")
public class PineapplesAutoRed extends OpMode {


    /* ================= CONSTANTS 1 ================= */
    double NORMAL_SPEED = 0.95;
    double SLOW_SPEED = 0.45;
    double INTAKE_RPM = 1300;
    long PRE_SHOOT_DELAY_MS = 0;
    long SHOOT_WAIT_MS = 0;
    long INTAKE_WAIT_MS = 300;
    long SECOND_INTAKE_EXTRA_MS = 300;
    double DRIVE_POWER_NORMAL = 1.0;
    double DRIVE_POWER_INTAKE = 0.65;


    double MIDDLE_BASE_POSITION = 0.69;
    double RIGHT_BASE_POSITION = 0.79;
    double LEFT_BASE_POSITION = 0.78;
    double shootingConst = 1300;


    long shooterStableSince = 0;


    static final int SHOOTER_TOL = 10;
    static final int SHOOTER_STABLE_MS = 250;
    boolean arrived = false;




    private static final long ARRIVAL_STABLE_MS = 150; // robot must be idle for 150ms
    private long arrivedSince = 0; // tracks when robot first stops




    /* ================= RED POSES ================= */
    private static final Pose[] RED_POSES = {
            new Pose(109.6, 125.6, Math.toRadians(36.03)), // start
            new Pose(84.9, 109.7, Math.toRadians(-56.38)), // check limelight
            new Pose(96, 96, Math.toRadians(225)),  // shoot 1
            new Pose(113.1, 110.0, Math.toRadians(-90)), //align 2
            new Pose(113.1, 84.0, Math.toRadians(-90)), //collect 2
            new Pose(96, 96, Math.toRadians(225)),  // shoot 2
            new Pose(88.3, 61.4, Math.toRadians(-90)), //intermediate
            new Pose(109.1, 55.6, Math.toRadians(-90)), //align 3
            new Pose(109.1, 37.4, Math.toRadians(-90)), //collect 3
            new Pose(85.5, 105.0, Math.toRadians(-150.1))   // shoot 3
    };


    /* ================= INTERNAL ================= */
    private PathChain[] paths;
    private Follower follower;
    private int state = 0;
    private long stateStartTime = 0;
    private boolean[] arrivedStates = new boolean[20]; // track per-state arrival
    private int gateStep = 0;
    private boolean preShootDelayDone = false;
    private boolean firstShootPositionReached = false;
    private long shootPositionTime = 0;


    static final double M_UP = 0.785;
    static final double R_DOWN = 0.625;
    static final double L_DOWN = 0.615;
    static final double R_UP = 0.79;
    static final double L_UP = 0.78;


    private boolean advanced = false;
    private double shootingspeed = shootingConst;


    private long outtakeStartTime = 0;
    private boolean gateOpened = false;




    /* ================= SHOOTING LOGIC ================= */
    static class ShootStep {
        final String name;
        final long durationMs; // -1 = condition-based


        ShootStep(String name, long durationMs) {
            this.name = name;
            this.durationMs = durationMs;
        }
    }


    private ShootStep[] shootingSteps = {
            new ShootStep("aim", 1500),
            new ShootStep("prepare", 0),
            new ShootStep("check", 250),
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
    private int shootCycle = -1;


    /* ================= INIT ================= */
    protected Limelight3A limelight;


    @Override
    public void init() {
        Common.configRobot(hardwareMap, false);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(RED_POSES[0]);
        buildPaths();


        Common.radvance.setPosition(RIGHT_BASE_POSITION);
        Common.madvance.setPosition(MIDDLE_BASE_POSITION);
        Common.ladvance.setPosition(LEFT_BASE_POSITION);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
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


        Pose pose = follower.getPose();
        telemetry.addData("Robot X", pose.getX());
        telemetry.addData("Robot Y", pose.getY());
        telemetry.addData("Heading", Math.toDegrees(pose.getHeading()));
        telemetry.addData("State", state);
        telemetry.addData("Arrived", arrived);


        follower.update();


        // default intake
        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);


        // Shooting motors
        DcMotorEx s1 = (DcMotorEx) Common.shoot;
        DcMotorEx s2 = (DcMotorEx) Common.shoot2;
        if (!"outtake".equals(
                shootingSteps[shootingcurstep].name
        )) {
            s1.setVelocity(shootingspeed);
            s2.setVelocity(-shootingspeed);
        }




        // check arrival
        if (!arrived) {
            if (!follower.isBusy()) {
                if (arrivedSince == 0) arrivedSince = System.currentTimeMillis();
                else if (System.currentTimeMillis() - arrivedSince >= ARRIVAL_STABLE_MS) {
                    arrived = true;
                    stateStartTime = System.currentTimeMillis();
                    telemetry.addLine("Arrived at state pose (stable)");
                }
            } else {
                arrivedSince = 0; // reset if still moving
            }
        }


        switch (state) {
            case 1: // limelight check
                if (arrived) handleCheckLimelight();
                break;


            case 2: case 5: case 9: // shooting
                // Only shoot if robot has fully arrived and optionally waited PRE_SHOOT_DELAY_MS
                if (arrived && System.currentTimeMillis() - stateStartTime >= PRE_SHOOT_DELAY_MS) {
                    handleShoot();
                } else {
                    telemetry.addLine("Driving to shooting pose...");
                }
                break;


            case 3: case 6: case 7:  // driving to stack
                handleMovement();
                break;


            case 4: case 8: // intake
                handleIntake();
                break;
        }




        telemetry.update();
    }




    /* ================= MOVEMENT/SHOOT/INTAKE ================= */
    private void handleMovement() {
        Common.radvance.setPosition(RIGHT_BASE_POSITION);
        Common.ladvance.setPosition(LEFT_BASE_POSITION);
        Common.advancewheel.setPower(-1);
        if (!arrivedStates[state] && !follower.isBusy()) {
            arrivedStates[state] = true;
            stateStartTime = System.currentTimeMillis();
        }
        if (arrivedStates[state] && System.currentTimeMillis() - stateStartTime > 200) {
            advance();
        }
    }


    private void handleIntake() {
        if (!arrivedStates[state] && !follower.isBusy()) {
            arrivedStates[state] = true;
            stateStartTime = System.currentTimeMillis();
        }
        if (!arrivedStates[state]) return;


        long wait = INTAKE_WAIT_MS;
        if (state == 7) wait += SECOND_INTAKE_EXTRA_MS;


        if (System.currentTimeMillis() - stateStartTime >= wait) {
            advance();
        }
    }


    private void handleShoot() {
        if (stepStartTime == 0) stepStartTime = System.currentTimeMillis();


        shootingcurstep = Math.min(shootingcurstep, shootingSteps.length - 1);
        ShootStep step = shootingSteps[shootingcurstep];
        long elapsed = System.currentTimeMillis() - stepStartTime;


        double shooterVel1 = ((DcMotorEx) Common.shoot).getVelocity();
        double shooterVel2 = ((DcMotorEx) Common.shoot2).getVelocity();
        Common.advancewheel.setPower(1);
        advanced = false;


        switch (step.name) {
            case "aim":
                //shootingspeed = shootingConst;
                double turnCmd = limelightTurnCmd();
                if (limelightAligned()) {
                    if (aimStableSince == 0) aimStableSince = System.currentTimeMillis();
                    if (System.currentTimeMillis() - aimStableSince >= AIM_STABLE_MS) {
                        nextShootStep();
                        aimStableSince = 0;
                    }
                } else {
                    aimStableSince = 0;
                }
                if (elapsed >= step.durationMs) nextShootStep();
                break;


            case "prepare":
                //shootingspeed = shootingConst;
                follower.startTeleopDrive();
                Common.radvance.setPosition(R_UP);
                Common.ladvance.setPosition(L_UP);
                nextShootStep();
                break;


            case "check":
                boolean inRange = Math.abs(shooterVel1 - shootingspeed) <= SHOOTER_TOL &&
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
                if (!advanced && elapsed >= step.durationMs) nextShootStep();
                break;


            case "mup": Common.madvance.setPosition(M_UP); if (elapsed >= step.durationMs) nextShootStep(); break;
            case "mdown": Common.madvance.setPosition(MIDDLE_BASE_POSITION);
                if (elapsed >= step.durationMs) nextShootStep();
                break;
            case "rdown": Common.radvance.setPosition(R_DOWN); if (elapsed >= step.durationMs) nextShootStep(); break;
            case "ldown": Common.ladvance.setPosition(L_DOWN); if (elapsed >= step.durationMs) nextShootStep(); break;
            case "increase": shootingspeed += 10*step.durationMs; nextShootStep(); break;
            case "decrease": shootingspeed -= 10*step.durationMs; nextShootStep(); break;
            case "outtake":
                // Start outtake timing
                if (outtakeStartTime == 0) {
                    outtakeStartTime = System.currentTimeMillis();
                    gateOpened = false;
                }


                // Force outtake speed
                ((DcMotorEx) Common.shoot).setVelocity(400);
                ((DcMotorEx) Common.shoot2).setVelocity(-400);


                long outtakeElapsed = System.currentTimeMillis() - outtakeStartTime;


                // Wait BEFORE opening gate
                if (!gateOpened && outtakeElapsed >= 200) { // 👈 motor spin-up time
                    Common.madvance.setPosition(M_UP);
                    gateOpened = true;
                }


                // Total outtake duration
                if (outtakeElapsed >= 450) { // 👈 full eject time
                    outtakeStartTime = 0;
                    nextShootStep();
                }
                break;






            case "done":
                // Reset middle gate after shooting
                Common.madvance.setPosition(MIDDLE_BASE_POSITION);


                //shootCycle++; // increment cycle after done
                advance();
                break;
        }


        telemetry.update();
    }


    void nextShootStep() {
        shootingcurstep++;
        stepStartTime = System.currentTimeMillis();
    }


    /* ================= STATE MACHINE ================= */
    private void advance() {
        state++;
        if (state >= RED_POSES.length) return;
        goToState(state);
    }


    private void goToState(int s) {
        arrived = false;
        shootingcurstep = 0;
        stepStartTime = 0;


        // set power
        // Slow ONLY when driving INTO intake poses
        if (s == 4 || s == 8) {   // 👈 collect 2 AND collect 3
            follower.setMaxPower(DRIVE_POWER_INTAKE);
        } else {
            follower.setMaxPower(DRIVE_POWER_NORMAL);
        }


        // Drive to the pose for **all states except idle beyond paths**
        if (s < paths.length) {
            follower.followPath(paths[s], true);
        }


        // reset aim if shooting
        if (s == 2 || s == 5 || s == 8) {
            aimPrevError = 0;
            aimPrevTime = 0;
            shootCycle++;
            shootingcurstep = 0;
            configureShootingForTagAndCycle();
        }
    }




    private void buildPaths() {
        paths = new PathChain[RED_POSES.length];

        for (int i = 1; i < RED_POSES.length; i++) {
            paths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(RED_POSES[i - 1], RED_POSES[i]))
                    .setLinearHeadingInterpolation(
                            RED_POSES[i - 1].getHeading(),
                            RED_POSES[i].getHeading()
                    )
                    .build();
        }
    }



    // ================= LIMELIGHT =================
    static double AIM_TX_OFFSET_DEG = -2.0;
    static final double AIM_TX_TOL_DEG = 0.5;
    static double AIM_KD = 0.002;
    static double AIM_KP = 0.01;
    private double aimPrevError = 0.0;
    private long aimPrevTime = 0;
    static final double AIM_MAX_TURN = 0.12;
    static boolean AIM_INVERT = false;
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
        // Scan for tag if not locked yet
        if (!tagLocked) {
            int tagId = scanForTag();
            if (tagId != -1) {
                detectedTag = tagId;
                tagLocked = true;
            }
        }


        // Print tag info to telemetry
        if (detectedTag != -1) {
            telemetry.addData("Detected Tag ID", detectedTag);
        } else {
            telemetry.addData("Detected Tag ID", "None");
        }


        // Wait 500ms at this state before advancing
        if (stepStartTime == 0) stepStartTime = System.currentTimeMillis();
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


        double tx = r.getTx();
        double error = tx - AIM_TX_OFFSET_DEG;


        if (Math.abs(error) <= AIM_TX_TOL_DEG) {
            aimPrevError = 0;
            return 0.0;
        }


        long now = System.nanoTime();
        double dt = (aimPrevTime == 0) ? 0.02 : (now - aimPrevTime) / 1e9;
        dt = Math.max(dt, 0.01);
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
        return tags.get(0).getFiducialId() != 23;
    }


    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }


    private void configureShootingForTagAndCycle() {


        if (detectedTag == 21) {


            if (shootCycle == 0) {
                // FIRST shooting position logic for tag 21
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 1200),      // limelight gated
                        new ShootStep("outtake", 1200),
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
                       //new ShootStep("mup", 750),
                        //new ShootStep("check", 750),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 750),
                        new ShootStep("ldown", 700),
                        //new ShootStep("check", 1000),
                        //new ShootStep("outtake", 1200),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 1) {
                // SECOND shooting position logic for tag 21
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
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
                        new ShootStep("check", 1500),  // shooter stable gated
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
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
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
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
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
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
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
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 500),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 500),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 1) {
                // SECOND shooting position logic for tag 23
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 500),
                        new ShootStep("ldown", 700),
                        new ShootStep("check", 500),
                        new ShootStep("rdown", 700),
                        new ShootStep("done", -1)
                };
            }
            else if (shootCycle == 2) {
                shootingSteps = new ShootStep[] {
                        new ShootStep("aim", 500),      // limelight gated
                        new ShootStep("prepare", 0),   // immediate
                        new ShootStep("check", 1500),  // shooter stable gated
                        new ShootStep("mup", 750),
                        new ShootStep("check", 500),
                        new ShootStep("rdown", 700),
                        new ShootStep("check", 500),
                        new ShootStep("ldown", 700),
                        new ShootStep("done", -1)
                };
            }
        }
    }


}

