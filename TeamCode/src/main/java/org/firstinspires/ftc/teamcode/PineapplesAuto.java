package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
public abstract class PineapplesAuto extends OpMode {

    /* ---------------- ALLIANCE ---------------- */
    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    private static final double FIELD_SIZE_IN = 144.0;

    protected Pose mirrorBlueToRed(Pose bluePose) {
        return new Pose(
                FIELD_SIZE_IN - bluePose.getX(),
                bluePose.getY(),
                AngleUnit.normalizeRadians(Math.PI - bluePose.getHeading())
        );
    }

    /* ---------------- PATHING ---------------- */
    private Follower follower;
    private PathChain[] pathChains;
    private Pose[] poseArray;

    private int state = 1;
    private long stopStartTime = 0;

    /* ---------------- SPEEDS ---------------- */
    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED   = 0.08;

    /* ---------------- TIMING ---------------- */
    private static final long SHOOT_STOP_MS = 3000;

    /* ---------------- MOTOR SPEEDS ---------------- */
    private static final double INTAKE_RPM  = 1300;
    private static final double SHOOTER_RPM = 1900;

    /* ---------------- POSES ---------------- */
    protected static final Pose[] poseArrayBlue = {
            new Pose(21, 125, Math.toRadians(145)),
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (1)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 85 , Math.toRadians(-90)), // INTAKE
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (4)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 60 , Math.toRadians(-90)), // INTAKE
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (7)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 40 , Math.toRadians(-90)), // INTAKE
    };

    @Override
    public void init() {

        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (getAlliance() == Alliance.BLUE)
                    ? poseArrayBlue[i]
                    : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[0]);

        buildPaths();
    }

    @Override
    public void start() {
        state = 1;
        stopStartTime = 0;
        follower.followPath(pathChains[state], true);
    }

    @Override
    public void loop() {

        /* ---------- INTAKE ALWAYS ---------- */
        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);

        follower.update();

        /* ---------- SHOOTING STOP ---------- */
        if (isShootIndex(state)) {

            // spin shooters while stopped
            ((DcMotorEx) Common.shoot).setVelocity(SHOOTER_RPM);
            ((DcMotorEx) Common.shoot2).setVelocity(-SHOOTER_RPM);

            if (!follower.isBusy()) {
                if (stopStartTime == 0) {
                    stopStartTime = System.currentTimeMillis();
                }

                if (System.currentTimeMillis() - stopStartTime >= SHOOT_STOP_MS) {
                    stopStartTime = 0;
                    advanceState();
                }
            }
            return;
        }

        /* ---------- NON-SHOOT STATES ---------- */
        if (!follower.isBusy()) {
            advanceState();
        }
    }

    private void advanceState() {
        state++;

        if (state >= pathChains.length) {
            state = 1; // loop forever
        }

        follower.followPath(pathChains[state], true);
    }

    /* ---------------- PATH BUILDING ---------------- */
    private void buildPaths() {
        pathChains = new PathChain[poseArray.length];

        for (int i = 0; i < poseArray.length - 1; i++) {

            boolean slow = isIntakeIndex(i + 1);
            double speed = slow ? SLOW_SPEED : NORMAL_SPEED;

            pathChains[i + 1] = follower.pathBuilder()
                    .addPath(new BezierLine(poseArray[i], poseArray[i + 1]))
                    .setLinearHeadingInterpolation(
                            poseArray[i].getHeading(),
                            poseArray[i + 1].getHeading(),
                            speed
                    )
                    .build();
        }
    }

    /* ---------------- INDEX CHECKS ---------------- */
    private boolean isShootIndex(int index) {
        return index == 1 || index == 4 || index == 7;
    }

    private boolean isIntakeIndex(int index) {
        return index == 3 || index == 6 || index == 9;
    }
}



