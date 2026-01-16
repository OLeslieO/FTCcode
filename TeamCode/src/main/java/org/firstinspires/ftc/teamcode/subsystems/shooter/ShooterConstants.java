package org.firstinspires.ftc.teamcode.subsystems.shooter;

public enum ShooterConstants {
    SHOOTER_SLOW_VELOCITY(1140),
    SHOOTER_MID_VELOCITY(1340),
    SHOOTER_FAST_VELOCITY(1480),
    SHOOTER_AUTO_SLOW_VELOCITY(950),

    SHOOTER_IDLE_VELOCITY(500),

    SHOOTER_P(38),
    SHOOTER_I(0),
    SHOOTER_D(17),
    SHOOTER_F(12.7)
    ;

    public final double value;

    ShooterConstants(double value) {
        this.value = value;
    }
}