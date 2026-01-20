package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "NM - Pineapple Auto RED", group = "Autonomous")
public class PineapplesAutoRed extends OpMode {

    /* ================= CONSTANTS ================= */

    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED = 0.10;

    private static final double INTAKE_RPM = 1300;
    private static final double SHOOTER_RPM = 1400;

    private static final long PRE_SHOOT_DELAY_MS = 500;
    private static final long SHOOT_WAIT_MS = 700;
    private static final long INTAKE_WAIT_MS = 900;
    private static final long SECOND_INTAKE_EXTRA_MS = 500;

    /* ================= RED POSES ================= */

    private static final Pose[] RED_POSES = {
            new Pose(123, 125, Math.toRadians(35)),    // 0 start

            new Pose(86, 85, Math.toRadians(225)),     // 1 shoot 1
            new Pose(120, 120, Math.toRadians(-90)),   // 2 move
            new Pose(120, 85, Math.toRadians(-90)),    // 3 intake 1

            new Pose(86, 85, Math.toRadians(225)),     // 4 shoot 2
            new Pose(120, 120, Math.toRadians(-90)),   // 5 move
            new Pose(120, 58, Math.toRadians(-90)),    // 6 intake 2

            new Pose(86, 85, Math.toRadians(225)),     // 7 shoot 3
            new Pose(120, 120, Math.toRadians(-90)),   // 8 move
            new Pose(120, 40, Math.toRadians(-90)),    // 9 intake 3
            new Pose(120, 40, Math.toRadians(-90))     // 10 FINAL SHOOT (same spot)
    };

    /* ================= INTERNAL ================= */

    private PathChain[] paths;
    private Follower follower;

    private int state = 0;
    private long stateStartTime = 0;
    private boolean arrived = false;
    private int gateStep = 0;
    private boolean preShootDelayDone = false;


    private void startShooter(){
        ((DcMotorEx) Common.shoot).setVelocity(SHOOTER_RPM);
        ((DcMotorEx) Common.shoot2).setVelocity(-SHOOTER_RPM);
    }
    private void stopShooter(){
        ((DcMotorEx) Common.shoot).setVelocity(0);
        ((DcMotorEx) Common.shoot2).setVelocity(0);
    }
    /* ================= INIT ================= */

    @Override
    public void init() {
        Common.configRobot(hardwareMap, false);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(RED_POSES[0]);

        buildPaths();

        Common.ladvance.setPosition(0.77);
        Common.madvance.setPosition(0.57);
        Common.radvance.setPosition(0.77);
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
        if (!arrived && !follower.isBusy()) {
            arrived = true;
            stateStartTime = System.currentTimeMillis();
            return;
        }

        if (!arrived) return;

        if (!preShootDelayDone) {
            if (System.currentTimeMillis() - stateStartTime < PRE_SHOOT_DELAY_MS) return;
            preShootDelayDone = true;
            stateStartTime = System.currentTimeMillis();
            return;
        }

        if (System.currentTimeMillis() - stateStartTime < SHOOT_WAIT_MS) return;

        DcMotorEx s1 = (DcMotorEx) Common.shoot;
        DcMotorEx s2 = (DcMotorEx) Common.shoot2;

        s1.setVelocity(SHOOTER_RPM);
        s2.setVelocity(-SHOOTER_RPM);

        switch (gateStep) {
            case 0: Common.madvance.setPosition(0.718); break;
            case 1: Common.ladvance.setPosition(0.58);
                    break;
            case 2: Common.radvance.setPosition(0.58);
                    Common.ladvance.setPosition(0.58);
            break;
            default:
                Common.ladvance.setPosition(0.77);
                Common.radvance.setPosition(0.77);
                Common.madvance.setPosition(0.57);
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
        preShootDelayDone = false;

        if (isShootState(s)) {
            startShooter();
        } else {
            stopShooter();
        }

        follower.followPath(paths[s], true);
    }



    /* ================= HELPERS ================= */

    private boolean isShootState(int s) {
        return s == 1 || s == 4 || s == 7 || s == 10;
    }

    private boolean isIntakeState(int s) {
        return s == 3 || s == 6 || s == 9;
    }

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
}
