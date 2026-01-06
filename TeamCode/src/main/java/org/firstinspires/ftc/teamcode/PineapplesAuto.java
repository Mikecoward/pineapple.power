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

    private int state = 0; // start at state 0
    private long stopStartTime = 0;
    private int gateStep = 0;

    /* ---------------- SPEEDS ---------------- */
    private static final double NORMAL_SPEED = 0.18;
    private static final double SLOW_SPEED = 0.12;
    private static final double INTAKE_SPEED_MODIFIER = 0.85;

    /* ---------------- MOTOR SPEEDS ---------------- */
    private static final double INTAKE_RPM = 1300;
    private static final double SHOOTER_RPM = 1325; // perfect speed for all balls

    /* ---------------- POSES ---------------- */
    protected static final Pose[] poseArrayBlue = {
            new Pose(21, 125, Math.toRadians(145)), // 0 start
            new Pose(58, 85 , Math.toRadians(-45)),  // 1 SHOOT 1
            new Pose(24, 120 , Math.toRadians(-90)), // 2 move to intake 1
            new Pose(24, 85 , Math.toRadians(-90)),  // 3 INTAKE 1
            new Pose(58, 85 , Math.toRadians(-45)),  // 4 SHOOT 2
            new Pose(24, 120 , Math.toRadians(-90)), // 5 move to intake 2
            new Pose(24, 58 , Math.toRadians(-90)),  // 6 INTAKE 2
            new Pose(58, 85 , Math.toRadians(-45)),  // 7 SHOOT 3
            new Pose(24, 120 , Math.toRadians(-90)), // 8 move to intake 3
            new Pose(24, 40 , Math.toRadians(-90)),  // 9 INTAKE 3
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
        state = 0;
        stopStartTime = 0;
        gateStep = 0;
        follower.followPath(pathChains[state + 1], true);
    }

    @Override
    public void loop() {

        ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);
        follower.update();

        if (isShootState(state)) {
            handleShooting();
            return;
        }

        if (isMoveToIntakeState(state)) {
            // robot slows down while moving to intake
            if (!follower.isBusy()) {
                // reached alignment pose, start intake path
                follower.followPath(pathChains[state + 1], true);
            }
            return;
        }

        if (isIntakeState(state)) {
            // intake active, robot should be stopped
            ((DcMotorEx) Common.intaking).setVelocity(INTAKE_RPM);
            // wait a little before advancing
            if (stopStartTime == 0) stopStartTime = System.currentTimeMillis();
            if (System.currentTimeMillis() - stopStartTime >= 500) { // 0.5 sec intake
                stopStartTime = 0;
                advanceState();
            }
            return;
        }

        // advance if normal path finished
        if (!follower.isBusy()) {
            advanceState();
        }
    }

    private void handleShooting() {
        DcMotorEx shoot  = (DcMotorEx) Common.shoot;
        DcMotorEx shoot2 = (DcMotorEx) Common.shoot2;
        DcMotorEx intake = (DcMotorEx) Common.intaking;

        double[] shooterSteps = {1.0, 1.0, 1.0, 1.0};
        double[] gatePositions = {0.71, 0.71, 0.718, -1};
        long[] delays = {2000, 1500, 1500, 1500};

        if (!follower.isBusy()) {
            if (stopStartTime == 0) stopStartTime = System.currentTimeMillis();
            long elapsed = System.currentTimeMillis() - stopStartTime;
            int step = gateStep;

            if (step < shooterSteps.length) {
                intake.setVelocity(INTAKE_RPM);
                shoot.setVelocity(shooterSteps[step] * SHOOTER_RPM);
                shoot2.setVelocity(-shooterSteps[step] * SHOOTER_RPM);

                if (elapsed >= delays[step]) {
                    if (gatePositions[step] > 0) {
                        if (step == 0) Common.ladvance.setPosition(gatePositions[step]);
                        else if (step == 1) Common.radvance.setPosition(gatePositions[step]);
                        else if (step == 2) Common.madvance.setPosition(gatePositions[step]);
                    }
                    stopStartTime = System.currentTimeMillis();
                    gateStep++;
                }
            } else {
                // finished shooting
                Common.ladvance.setPosition(0.535);
                Common.radvance.setPosition(0.55);
                Common.madvance.setPosition(0.56);

                stopStartTime = 0;
                gateStep = 0;
                advanceState();
            }
        }
    }

    private void advanceState() {
        state++;
        if (state >= pathChains.length) state = pathChains.length - 1; // stop at last pose
        follower.followPath(pathChains[state], true);
    }

    private void buildPaths() {
        pathChains = new PathChain[poseArray.length];
        for (int i = 0; i < poseArray.length - 1; i++) {
            boolean slow = isMoveToIntakeState(i + 1) || isIntakeState(i + 1);
            double speed = slow ? SLOW_SPEED * INTAKE_SPEED_MODIFIER : NORMAL_SPEED;

            pathChains[i + 1] = follower.pathBuilder()
                    .addPath(new BezierLine(poseArray[i], poseArray[i + 1]))
                    .setLinearHeadingInterpolation(poseArray[i].getHeading(), poseArray[i + 1].getHeading(), speed)
                    .build();
        }
    }

    private boolean isShootState(int s) {
        return s == 1 || s == 5 || s == 9;
    }

    private boolean isMoveToIntakeState(int s) {
        return s == 2 || s == 6 || s == 10;
    }

    private boolean isIntakeState(int s) {
        return s == 3 || s == 7 || s == 11;
    }
}



