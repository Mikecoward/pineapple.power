package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "Pineapples Testing Speed", group = "Test")
public class PineapplesTestingSpeed extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    // ---------------- DRIVING ----------------
    double cmdX=0, cmdY=0, cmdTurn=0;
    static final double JOYSTICK_SLEW = 1;
    double driveSpeedCap = .8;
    static final double DEADBAND = 0.06;

    // ---------------- LOOKUP TABLE (LIVE EDIT) ----------------
    double[][] table = {
            {45,1225},
            {50,1275},
            {55,1280},
            {60,1300},
            {65,1350},
            {70,1375},
            {75,1400},
            {80,1425}
    };

    int selectedIndex = 0;
    static final double SPEED_STEP = 25;
    static final double DIST_STEP = 1;

    boolean upPrev, downPrev, leftPrev, rightPrev;

    // ---------------- SHOOTING STATE ----------------
    String curstep = "stagnant";
    int shootingcurstep = 0;
    long stepStartTime = 0;
    long shooterStableSince = 0;

    static final int SHOOTER_TOL = 10;
    static final int SHOOTER_STABLE_MS = 250;

    static final double M_UP = 0.785;
    static final double R_DOWN = 0.57;
    static final double L_DOWN = 0.58;
    static final double R_UP = 0.78;
    static final double L_UP = 0.78;

    double shootingspeed = 0;
    double intakingspeed = 1200;

    ShootStep[] shootingSteps = {
            new ShootStep("aim",-1),
            new ShootStep("prepare",0),
            new ShootStep("check",1500),
            new ShootStep("mup",750),
            new ShootStep("check",750),
            new ShootStep("rdown",700),
            new ShootStep("check",750),
            new ShootStep("ldown",700),
            new ShootStep("done",-1)
    };

    static class ShootStep {
        final String name;
        final long durationMs;
        ShootStep(String n,long d){ name=n; durationMs=d; }
    }

    @Override
    public void init() {
        Common.configRobot(hardwareMap,false);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));
        follower.update();

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        follower.update();
        Pose p = follower.getPose();
        double curx = p.getX();
        double cury = p.getY();
        double distance = distanceTOGOAL(curx,cury);

        // ---------------- DRIVING (unchanged) ----------------
        double rawY = deadband(gamepad1.right_stick_y);
        double rawX = deadband(gamepad1.right_stick_x);
        double rawTurn = deadband(gamepad1.left_stick_x);

        double targetY = expo(rawY,1.5);
        double targetX = expo(rawX,1.5);
        double targetTurn = expo(rawTurn,1.5)/1.5;

        cmdY += clamp(targetY-cmdY,-JOYSTICK_SLEW,JOYSTICK_SLEW);
        cmdX += clamp(targetX-cmdX,-JOYSTICK_SLEW,JOYSTICK_SLEW);

        if(targetTurn==0) cmdTurn=0;
        else cmdTurn += clamp(targetTurn-cmdTurn,-JOYSTICK_SLEW,JOYSTICK_SLEW);

        follower.setTeleOpDrive(
                -cmdY*driveSpeedCap,
                -cmdX*driveSpeedCap,
                -cmdTurn*driveSpeedCap,
                false
        );

        // ---------------- LOOKUP EDITING ----------------
        if(gamepad1.dpad_up && !upPrev)
            selectedIndex=(selectedIndex+1)%table.length;

        if(gamepad1.dpad_down && !downPrev)
            selectedIndex=(selectedIndex-1+table.length)%table.length;

        if(gamepad1.dpad_right && !rightPrev)
            table[selectedIndex][1]+=SPEED_STEP;

        if(gamepad1.dpad_left && !leftPrev)
            table[selectedIndex][1]-=SPEED_STEP;

        upPrev=gamepad1.dpad_up;
        downPrev=gamepad1.dpad_down;
        leftPrev=gamepad1.dpad_left;
        rightPrev=gamepad1.dpad_right;

        // ---------------- SHOOTING MODE TOGGLE ----------------
        curstep = gamepad1.right_bumper ? "shooting" : "stagnant";

        if("shooting".equals(curstep)){

            if(stepStartTime==0)
                stepStartTime=System.currentTimeMillis();

            shootingcurstep=Math.min(shootingcurstep,shootingSteps.length-1);
            ShootStep step=shootingSteps[shootingcurstep];
            long elapsed=System.currentTimeMillis()-stepStartTime;

            double shooterVel1=((DcMotorEx)Common.shoot).getVelocity();
            double shooterVel2=((DcMotorEx)Common.shoot2).getVelocity();

            switch(step.name){

                case "aim":
                    follower.setTeleOpDrive(0,0,0,false);
                    shootingspeed=lookupSpeed(distance);
                    if(elapsed>400) nextStep();
                    break;

                case "prepare":
                    shootingspeed=lookupSpeed(distance);
                    nextStep();
                    break;

                case "check":
                    boolean inRange=
                            Math.abs(shooterVel1-shootingspeed)<=SHOOTER_TOL &&
                                    Math.abs(shooterVel2+shootingspeed)<=SHOOTER_TOL;

                    if(inRange){
                        if(shooterStableSince==0)
                            shooterStableSince=System.currentTimeMillis();
                        if(System.currentTimeMillis()-shooterStableSince>=SHOOTER_STABLE_MS)
                            nextStep();
                    }else shooterStableSince=0;

                    if(elapsed>=step.durationMs) nextStep();
                    break;

                case "mup":
                    Common.madvance.setPosition(M_UP);
                    if(elapsed>=step.durationMs) nextStep();
                    break;

                case "rdown":
                    Common.radvance.setPosition(R_DOWN);
                    if(elapsed>=step.durationMs) nextStep();
                    break;

                case "ldown":
                    Common.ladvance.setPosition(L_DOWN);
                    if(elapsed>=step.durationMs) nextStep();
                    break;

                case "done":
                    break;
            }

        } else {

            shootingcurstep=0;
            stepStartTime=0;
            shootingspeed=0;
            shooterStableSince=0;
            Common.advancewheel.setPower(-1);
        }

        // ---------------- MOTORS ----------------
        ((DcMotorEx)Common.intaking).setVelocity(1200);
        ((DcMotorEx)Common.shoot).setVelocity(shootingspeed);
        ((DcMotorEx)Common.shoot2).setVelocity(-shootingspeed);

        // ---------------- TELEMETRY ----------------
        telemetry.addLine("=== SPEED TUNING MODE ===");
        telemetry.addData("Distance", "%.1f", distance);
        telemetry.addData("cmdY", "%.3f", cmdY);
        telemetry.addData("cmdX", "%.3f", cmdX);
        telemetry.addData("cmdTurn", "%.3f", cmdTurn);

        telemetry.addData("State", curstep);
        telemetry.addData("Shoot Step", shootingSteps[Math.min(shootingcurstep,shootingSteps.length-1)].name);
        telemetry.addData("Shooter Target", shootingspeed);
        telemetry.addData("Shooter v1", shooterVel1());
        telemetry.addData("Shooter v2", shooterVel2());

        telemetry.addLine("---- Lookup Table ----");
        for(int i=0;i<table.length;i++){
            String mark=(i==selectedIndex)?" <==":"";
            telemetry.addData("Row "+i,
                    "Dist %.1f | Speed %.0f%s",
                    table[i][0],table[i][1],mark);
        }

        telemetry.update();
    }

    private void nextStep(){
        shootingcurstep++;
        stepStartTime=System.currentTimeMillis();
    }

    private double lookupSpeed(double distance){
        double closest=table[0][1];
        double smallest=Math.abs(distance-table[0][0]);
        for(int i=1;i<table.length;i++){
            double err=Math.abs(distance-table[i][0]);
            if(err<smallest){
                smallest=err;
                closest=table[i][1];
            }
        }
        return closest;
    }

    private double distanceTOGOAL(double x,double y){
        return Math.hypot(128-x,128-y);
    }

    private double shooterVel1(){
        return ((DcMotorEx)Common.shoot).getVelocity();
    }

    private double shooterVel2(){
        return ((DcMotorEx)Common.shoot2).getVelocity();
    }

    private double deadband(double v){
        return Math.abs(v)<DEADBAND?0:v;
    }

    private double expo(double input,double exponent){
        return Math.copySign(Math.pow(Math.abs(input),exponent),input);
    }

    private double clamp(double value,double min,double max){
        return Math.max(min,Math.min(max,value));
    }
}
