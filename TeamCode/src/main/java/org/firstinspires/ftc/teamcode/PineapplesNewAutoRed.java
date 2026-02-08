package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PineappleBot New Auto RED", group = "Autonomous")
public class PineapplesNewAutoRed extends PineapplesBaseAuto {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}