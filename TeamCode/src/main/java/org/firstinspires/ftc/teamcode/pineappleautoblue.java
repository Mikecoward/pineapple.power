package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NM - Pineapple Auto BLUE", group = "Autonomous")
public class pineappleautoblue extends PineapplesAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}