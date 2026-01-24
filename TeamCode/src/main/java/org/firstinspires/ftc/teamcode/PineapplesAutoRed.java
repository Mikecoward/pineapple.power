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
@Autonomous(name = "NM - Pineapple Auto RED Stationary Shoot", group = "Autonomous")
public class PineapplesAutoRed extends OpMode {

    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED = 0.10;

    private static final double INTAKE_RPM = 1300;
    private static final double SHOOTER_RPM = 1400;

    private static final long PRE_SHOOT_DELAY_MS = 1000;
    private static final long SHOOT_WAIT_MS = 700;
    private static final long INTAKE_WAIT_MS = 900;
    private static final long SECOND_INTAKE_EXTRA_MS = 500;

    private static final long M_PUSH_TIME = 500;
    private static final long SIDE_PUSH_TIME = 500;
    private static final long SHOOT_HOLD_MS = 2000; // hold shooting position 2 seconds

    private static final Pose[] RED_POSES = {
            new Pose(123, 125, Math.toRadians(35)),
            new Pose(86, 85, Math.toRadians(225)),
            new Pose(120, 120, Math.toRadians(-90)),
            new Pose(120, 85, Math.toRadians(-90)),
            new Pose(86, 85, Math.toRadians(225)),
            new Pose(120, 120, Math.toRadians(-90)),
            new Pose(120, 58, Math.toRadians(-90)),
            new Pose(86, 85, Math.toRadians(225)),
            new Pose(118, 120, Math.toRadians(-90)),
            new Pose(118, 40, Math.toRadians(-90)),
            new Pose(118, 40, Math.toRadians(-90))
    };

    private PathChain[] paths;
    private Follower follower;

    private int state = 0;
    private long stateStartTime = 0;
    private boolean arrived = false;
    private int gateStep = 0;
    private boolean preShootDelayDone = false;

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

        // STOP ROBOT COMPLETELY while shooting
        follower.setTeleOpDrive(0,0,0,false);

        ((DcMotorEx) Common.shoot).setVelocity(SHOOTER_RPM);
        ((DcMotorEx) Common.shoot2).setVelocity(-SHOOTER_RPM);

        // GATE SEQUENCE
        switch (gateStep) {
            case 0:
                Common.madvance.setPosition(0.718); // push middle gate
                if (System.currentTimeMillis() - stateStartTime > M_PUSH_TIME) gateStep++;
                break;
            case 1:
                Common.ladvance.setPosition(0.66); // left gate push
                if (System.currentTimeMillis() - stateStartTime > SIDE_PUSH_TIME) gateStep++;
                break;
            case 2:
                Common.radvance.setPosition(0.66); // right gate push
                if (System.currentTimeMillis() - stateStartTime > SIDE_PUSH_TIME) gateStep++;
                break;
            case 3:
                Common.madvance.setPosition(0.718); // middle gate again for last ball
                if (System.currentTimeMillis() - stateStartTime > M_PUSH_TIME) gateStep++;
                break;
            case 4:
                // HOLD SHOOTING POSITION for a moment
                if (System.currentTimeMillis() - stateStartTime > SHOOT_HOLD_MS) {
                    Common.ladvance.setPosition(0.77);
                    Common.madvance.setPosition(0.57);
                    Common.radvance.setPosition(0.77);
                    gateStep = 0;
                    advance();
                }
                break;
        }
    }

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
            ((DcMotorEx) Common.shoot).setVelocity(SHOOTER_RPM);
            ((DcMotorEx) Common.shoot2).setVelocity(-SHOOTER_RPM);
        } else {
            ((DcMotorEx) Common.shoot).setVelocity(0);
            ((DcMotorEx) Common.shoot2).setVelocity(0);
        }

        follower.followPath(paths[s], true);
    }

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
