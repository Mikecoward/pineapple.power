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
public abstract class PineapplesAuto extends OpMode {

    public enum Alliance { BLUE, RED }
    protected abstract Alliance getAlliance();

    private static final double FIELD_SIZE_IN = 144.0;

    protected Pose mirrorBlueToRed(Pose bluePose) {
        return new Pose(
                FIELD_SIZE_IN - bluePose.getX(),
                bluePose.getY(),
                Math.PI - bluePose.getHeading()
        );
    }

    /* ---------------- PATHING ---------------- */
    private Follower follower;
    private PathChain[] pathChains;
    private Pose[] poseArray;

    private int state = 1;
    private long stopStartTime = 0;
    private int gateStep = 0; // 0 = not started, 1 = L, 2 = R, 3 = M

    /* ---------------- SPEEDS ---------------- */
    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED   = 0.08;

    /* ---------------- TIMING ---------------- */
    private static final long SHOOT_STOP_MS = 3000;
    private static final long GATE_DELAY_MS = 500;

    /* ---------------- MOTOR SPEEDS ---------------- */
    private static final double INTAKE_RPM  = 1300;
    private static final double SHOOTER_RPM = 1325;

    /* ---------------- POSES ---------------- */
    protected static final Pose[] poseArrayBlue = {
            new Pose(21, 125, Math.toRadians(145)),
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (1)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 85 , Math.toRadians(-90)), // INTAKE
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (4)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 58 , Math.toRadians(-90)), // INTAKE updated for better collection
            new Pose(58, 85 , Math.toRadians(-45)), // SHOOT (7)
            new Pose(24, 120 , Math.toRadians(-90)),
            new Pose(24, 40 , Math.toRadians(-90)), // INTAKE
    };

    @Override
    public void init() {
        Common.configRobot(hardwareMap, false);

        poseArray = new Pose[poseArrayBlue.length];
        for (int i = 0; i < poseArrayBlue.length; i++) {
            poseArray[i] = (getAlliance() == Alliance.BLUE)
                    ? poseArrayBlue[i]
                    : mirrorBlueToRed(poseArrayBlue[i]);
        }

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poseArray[0]);

        buildPaths();

        // close gates at start
        Common.ladvance.setPosition(0.535);
        Common.madvance.setPosition(0.56);
        Common.radvance.setPosition(0.55);
    }

    @Override
    public void start() {
        state = 1;
        stopStartTime = 0;
        gateStep = 0;
        follower.followPath(pathChains[state], true);
    }

    @Override
    public void loop() {

        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);

        follower.update();

        if (isShootIndex(state)) {
            handleShooting();
            return;
        }

        if (!follower.isBusy()) {
            advanceState();
        }
    }

    private void handleShooting() {
        DcMotorEx shoot  = (DcMotorEx) Common.shoot;
        DcMotorEx shoot2 = (DcMotorEx) Common.shoot2;
        DcMotorEx intake = (DcMotorEx) Common.intaking;

        // ramp shooter wheel in steps like teleop
        double[] shooterSteps = {0.88, 0.84, 0.98, 1.0};
        double[] gatePositions = {0.71, 0.71, 0.718, -1}; // last -1 means no gate change
        long[] delays = {2000, 1500, 1500, 1500};

        if (!follower.isBusy()) {
            if (stopStartTime == 0) stopStartTime = System.currentTimeMillis();

            long elapsed = System.currentTimeMillis() - stopStartTime;

            int step = gateStep; // use gateStep as step counter for shooting sequence

            if (step < shooterSteps.length) {
                intake.setVelocity(INTAKE_RPM);
                shoot.setVelocity(shooterSteps[step] * SHOOTER_RPM);
                shoot2.setVelocity(-shooterSteps[step] * SHOOTER_RPM);

                // wait for timeout of current step
                if (elapsed >= delays[step]) {
                    // move corresponding gate if needed
                    if (gatePositions[step] > 0) {
                        if (step == 0) Common.ladvance.setPosition(gatePositions[step]);
                        else if (step == 1) Common.radvance.setPosition(gatePositions[step]);
                        else if (step == 2) Common.madvance.setPosition(gatePositions[step]);
                    }
                    stopStartTime = System.currentTimeMillis();
                    gateStep++;
                }
            } else {
                // finished all steps, close gates
                Common.ladvance.setPosition(0.535);
                Common.radvance.setPosition(0.55);
                Common.madvance.setPosition(0.56);

                // reset for next shooting point
                stopStartTime = 0;
                gateStep = 0;
                advanceState();
            }
        }
    }

    private void advanceState() {
        state++;
        if (state >= pathChains.length) state = 1;
        follower.followPath(pathChains[state], true);
    }

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

    private boolean isShootIndex(int index) {
        return index == 1 || index == 4 || index == 7;
    }

    private boolean isIntakeIndex(int index) {
        return index == 3 || index == 6 || index == 9;
    }
}



