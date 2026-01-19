package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public abstract class PineapplesAutoBlue extends OpMode {

    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    /* ================= CONSTANTS ================= */

    private static final double FIELD_SIZE_IN = 144.0;

    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED = 0.10;

    private static final double INTAKE_RPM = 1300;

    private static final long PRE_SHOOT_DELAY_MS = 1000; // ⭐ NEW: wait after arrival
    private static final long SHOOT_WAIT_MS = 700;
    private static final long INTAKE_WAIT_MS = 900;
    private static final long SECOND_INTAKE_EXTRA_MS = 500;

    /* ================= POSES ================= */

    protected static final Pose[] BLUE_POSES = {
            new Pose(21, 125, Math.toRadians(145)), // 0 start

            new Pose(58, 85, Math.toRadians(-45)),  // 1 shoot 1
            new Pose(23, 120, Math.toRadians(-90)), // 2 intake move
            new Pose(23, 85, Math.toRadians(-90)),  // 3 intake 1

            new Pose(58, 85, Math.toRadians(-45)),  // 4 shoot 2
            new Pose(21, 120, Math.toRadians(-90)), // 5 intake move
            new Pose(21, 58, Math.toRadians(-90)),  // 6 intake 2

            new Pose(58, 85, Math.toRadians(-45)),  // 7 shoot 3
            new Pose(21, 120, Math.toRadians(-90)), // 8 intake move
            new Pose(21, 40, Math.toRadians(-90))   // 9 intake 3
    };

    /* ================= INTERNAL ================= */

    private Pose[] poses;
    private PathChain[] paths;
    private Follower follower;

    private int state = 0;
    private long stateStartTime = 0;
    private boolean arrived = false;
    private int gateStep = 0;
    private boolean preShootDelayDone = false; // ⭐ NEW

    /* ================= MIRROR ================= */

    protected Pose mirrorBlueToRed(Pose p) {
        return new Pose(
                FIELD_SIZE_IN - p.getX(),
                p.getY(),
                Math.PI - p.getHeading()
        );
    }

    /* ================= INIT ================= */

    @Override
    public void init() {
        Common.configRobot(hardwareMap, false);

        poses = new Pose[BLUE_POSES.length];
        for (int i = 0; i < BLUE_POSES.length; i++) {
            poses[i] = (getAlliance() == Alliance.BLUE)
                    ? BLUE_POSES[i]
                    : mirrorBlueToRed(BLUE_POSES[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses[0]);

        buildPaths();

        Common.ladvance.setPosition(0.535);
        Common.madvance.setPosition(0.56);
        Common.radvance.setPosition(0.55);
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
        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);

        if (isShootState(state)) {
            handleShoot();
            return;
        }

        if (isIntakeState(state)) {
            handleIntake();
            return;
        }

        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
        }

        if (arrived && System.currentTimeMillis() - stateStartTime > 200) {
            advance();
        }
    }

    /* ================= SHOOT ================= */

    private void handleShoot() {

        // Detect arrival
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
            return;
        }

        if (!arrived) return;

        // ⭐ One-time delay before first gate
        if (!preShootDelayDone) {
            if (System.currentTimeMillis() - stateStartTime < PRE_SHOOT_DELAY_MS) {
                return;
            }
            preShootDelayDone = true;
            stateStartTime = System.currentTimeMillis();
            return;
        }

        // Wait between gate actions
        if (System.currentTimeMillis() - stateStartTime < SHOOT_WAIT_MS) return;

        DcMotorEx s1 = (DcMotorEx) Common.shoot;
        DcMotorEx s2 = (DcMotorEx) Common.shoot2;

        double rpm = 1250;
        s1.setVelocity(rpm);
        s2.setVelocity(-rpm);

        if (gateStep == 0) {
            Common.ladvance.setPosition(0.71);
        } else if (gateStep == 1) {
            Common.radvance.setPosition(0.71);
        } else if (gateStep == 2) {
            Common.madvance.setPosition(0.718);
        } else {
            Common.ladvance.setPosition(0.535);
            Common.radvance.setPosition(0.55);
            Common.madvance.setPosition(0.56);
            gateStep = 0;
            advance();
            return;
        }

        gateStep++;
        stateStartTime = System.currentTimeMillis();
    }

    /* ================= INTAKE ================= */

    private void handleIntake() {
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
        }

        if (!arrived) return;

        long wait = INTAKE_WAIT_MS;
        if (state == 6) wait += SECOND_INTAKE_EXTRA_MS;

        if (System.currentTimeMillis() - stateStartTime >= wait) {
            advance();
        }
    }

    /* ================= STATE ================= */

    private void advance() {
        state++;
        if (state >= paths.length) return;
        goToState(state);
    }

    private void goToState(int s) {
        arrived = false;
        gateStep = 0;
        preShootDelayDone = false; // ⭐ reset every shooting state
        follower.followPath(paths[s], true);
    }

    /* ================= HELPERS ================= */

    private boolean isShootState(int s) {
        return s == 1 || s == 4 || s == 7;
    }

    private boolean isIntakeState(int s) {
        return s == 3 || s == 6 || s == 9;
    }

    private void buildPaths() {
        paths = new PathChain[poses.length];
        for (int i = 1; i < poses.length; i++) {
            boolean slow = isIntakeState(i);
            double speed = slow ? SLOW_SPEED : NORMAL_SPEED;

            paths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(poses[i - 1], poses[i]))
                    .setLinearHeadingInterpolation(
                            poses[i - 1].getHeading(),
                            poses[i].getHeading(),
                            speed
                    )
                    .build();
        }
    }
}


