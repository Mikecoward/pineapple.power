package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "NM - Pineapple Auto RED SHOOT", group = "Autonomous")
public class PineapplesAutoRed extends OpMode {

    /* ================= CONSTANTS ================= */
    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED = 0.10;
    private static final double INTAKE_RPM = 1300;
    private static final long PRE_SHOOT_DELAY_MS = 1000;
    private static final long SHOOT_WAIT_MS = 700;
    private static final long INTAKE_WAIT_MS = 900;
    private static final long SECOND_INTAKE_EXTRA_MS = 500;
    private static final double DRIVE_POWER_NORMAL = 1.0;
    private static final double DRIVE_POWER_INTAKE = 0.45; // tune this

    /* ================= RED POSES ================= */
    private static final Pose[] RED_POSES = {
            new Pose(123, 125, Math.toRadians(35)),    // start
            new Pose(86, 85, Math.toRadians(225)),     // shoot 1
            new Pose(120, 120, Math.toRadians(-90)),
            new Pose(120, 85, Math.toRadians(-90)),
            new Pose(86, 85, Math.toRadians(225)),     // shoot 2
            new Pose(119, 85, Math.toRadians(-90)),
            new Pose(119, 58, Math.toRadians(-90)),
            new Pose(86, 85, Math.toRadians(225)),     // shoot 3
            new Pose(119, 58, Math.toRadians(-90)),
            new Pose(119, 40, Math.toRadians(-90))
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

    /* ================= SHOOTING LOGIC ================= */
    private enum ShootStepType { AIM, PREPARE, CHECK, M_UP, R_DOWN, L_DOWN, DONE }

    private class ShootStep {
        ShootStepType type;
        long durationMs;
        ShootStep(ShootStepType t, long ms) { type = t; durationMs = ms; }
    }

    private ShootStep[] shootingSteps = {
            new ShootStep(ShootStepType.AIM, -1),      // limelight gated
            new ShootStep(ShootStepType.PREPARE, 0),   // immediate
            new ShootStep(ShootStepType.CHECK, 1500),  // shooter stable gated
            new ShootStep(ShootStepType.M_UP, 750),
            new ShootStep(ShootStepType.R_DOWN, 700),
            new ShootStep(ShootStepType.L_DOWN, 700),
            new ShootStep(ShootStepType.DONE, -1)
    };

    private int shootingcurstep = 0;
    private long stepStartTime = 0;

    /* ================= INIT ================= */
    protected Limelight3A limelight; // copy this from teleop

    @Override
    public void init() {
        // existing init code...
        Common.configRobot(hardwareMap, false);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(RED_POSES[0]);
        buildPaths();

        // gates neutral
        Common.ladvance.setPosition(0.77);
        Common.madvance.setPosition(0.64);
        Common.radvance.setPosition(0.77);

        // --- LIMELIGHT INIT ---
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
        follower.update();

        // default intake running
        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);

        // handle shooting states
        if (isShootState(state)) {
            handleShoot();
            return;
        }

        // handle intake states
        if (isIntakeState(state)) {
            handleIntake();
            return;
        }

        // movement logic
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
        }

        if (arrived && System.currentTimeMillis() - stateStartTime > 200) {
            advance();
        }
    }

    /* ================= SHOOT HANDLER ================= */
    private void handleShoot() {
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
            return;
        }

        if (!arrived) return;

        if (stepStartTime == 0) stepStartTime = System.currentTimeMillis();

        ShootStep step = shootingSteps[shootingcurstep];
        long elapsed = System.currentTimeMillis() - stepStartTime;

        DcMotorEx s1 = (DcMotorEx) Common.shoot;
        DcMotorEx s2 = (DcMotorEx) Common.shoot2;

        // Check if it's the first shooting position (adjust the position as needed)
        if (!firstShootPositionReached && state == 1) {
            firstShootPositionReached = true;
            shootPositionTime = System.currentTimeMillis();
        }

        // Wait for 1 second after reaching the first shooting position before shooting
        if (firstShootPositionReached && System.currentTimeMillis() - shootPositionTime < 1000) {
            return; // Wait for 1 second
        }

        switch (step.type) {
            case AIM:
                // use limelight to align
                double turnCmd = limelightTurnCmd();
                follower.setTeleOpDrive(0, 0, -turnCmd, true);
                if (limelightAligned()) nextShootStep();
                break;

            case PREPARE:
                Common.advancewheel.setPower(1);
                Common.radvance.setPosition(0.77);
                Common.madvance.setPosition(0.785);
                Common.ladvance.setPosition(0.77);
                nextShootStep();
                break;

            case CHECK:
                s1.setVelocity(1400);
                s2.setVelocity(-1400);
                boolean inRange = Math.abs(s1.getVelocity() - 1400) < 20 && Math.abs(s2.getVelocity() + 1400) < 20;
                if (inRange) nextShootStep();
                else if (elapsed >= step.durationMs) nextShootStep();
                break;

            case M_UP:
                Common.madvance.setPosition(0.785);
                if (elapsed >= step.durationMs) nextShootStep();
                break;

            case R_DOWN:
                Common.radvance.setPosition(0.57);
                if (elapsed >= step.durationMs) nextShootStep();
                break;

            case L_DOWN:
                Common.ladvance.setPosition(0.58);
                if (elapsed >= step.durationMs) nextShootStep();
                break;

            case DONE:
                // hold shooter velocity
                s1.setVelocity(1400);
                s2.setVelocity(-1400);

                // return all advance servos to neutral
                Common.ladvance.setPosition(0.77);
                Common.madvance.setPosition(0.785);  // reset middle
                Common.radvance.setPosition(0.77);   // reset right

                // wait 500ms before moving to next path
                if (!preShootDelayDone) {
                    stepStartTime = System.currentTimeMillis();
                    preShootDelayDone = true;
                } else if (System.currentTimeMillis() - stepStartTime >= 500) {
                    Common.madvance.setPosition(0.64);
                    advance();  // go to next path
                }
                break;
        }
    }

    private void nextShootStep() {
        shootingcurstep++;
        stepStartTime = System.currentTimeMillis();
        if (shootingcurstep >= shootingSteps.length) shootingcurstep = shootingSteps.length - 1;
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
        if (state >= paths.length) return;
        goToState(state);
    }

    private void goToState(int s) {
        arrived = false;
        gateStep = 0;
        preShootDelayDone = false;
        shootingcurstep = 0;
        stepStartTime = 0;

        // 🔽 SLOW DOWN WHEN INTAKING
        if (isIntakeState(s)) {
            follower.setMaxPower(DRIVE_POWER_INTAKE);
        } else {
            follower.setMaxPower(DRIVE_POWER_NORMAL);
        }

        follower.followPath(paths[s], true);
    }

    private boolean isShootState(int s) { return s == 1 || s == 4 || s == 7; }
    private boolean isIntakeState(int s) { return s == 3 || s == 6 || s == 9; }

    private void buildPaths() {
        paths = new PathChain[RED_POSES.length];
        for (int i = 1; i < RED_POSES.length; i++) {
            boolean slow = isIntakeState(i);
            double speed = slow ? SLOW_SPEED : NORMAL_SPEED;
            paths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(RED_POSES[i - 1], RED_POSES[i]))
                    .setLinearHeadingInterpolation(
                            RED_POSES[i - 1].getHeading(),
                            RED_POSES[i].getHeading(),
                            speed
                    )
                    .build();
        }
    }

    // LIMELIGHT CODE:
    static final double AIM_TX_OFFSET_DEG = 0.0;
    static final double AIM_TX_TOL_DEG = 1.0;
    static final double AIM_KP = 0.01;      // stronger P, but no min power
    static final double AIM_MAX_TURN = 0.12;
    static boolean AIM_INVERT = false;       // flip ONLY if needed

    private double limelightTurnCmd() {
        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;

        double tx = r.getTx();                 // degrees
        double error = tx;                     // no offset

        // Stop if we're close enough
        if (Math.abs(error) <= AIM_TX_TOL_DEG) {
            return 0.0;
        }

        // Simple proportional control
        double turn = AIM_KP * error;

        // Clamp max speed
        turn = clamp(turn, -AIM_MAX_TURN, AIM_MAX_TURN);

        return AIM_INVERT ? -turn : turn;
    }

    protected double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean limelightAligned() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.isValid() && Math.abs(r.getTx() - AIM_TX_OFFSET_DEG) <= AIM_TX_TOL_DEG;
    }
}
