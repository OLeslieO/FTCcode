package org.firstinspires.ftc.teamcode.subsystems.ballstorage;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class BallStorage {

    /* ===================== 硬件 ===================== */

    private final RevColorSensorV3 colorSensor;

    private final Rev2mDistanceSensor distanceSensor;

    /* ===================== HSV 缓存 ===================== */

    private final float[] hsv = new float[3];
    private int lastR, lastG, lastB;

    /* ===================== 判定参数（可比赛调） ===================== */

    // Green
    public static double GREEN_MIN_H = 150;
    public static double GREEN_MAX_H = 170;
    public static double GREEN_MIN_S = 0.6;
    public static double GREEN_MIN_V = 0.0;

    // Purple
    public static double PURPLE_MIN_H = 205;
    public static double PURPLE_MAX_H = 240;
    public static double PURPLE_MIN_S = 0.4;

    // 抗抖
    private static final int MIN_IN_ZONE_FRAMES = 3;

    // 容量
    private static final int MAX_BALLS = 3;

    /* ===================== 内部状态 ===================== */

    private final List<Float> hueSamples = new ArrayList<>();
    private final Queue<Integer> colorQueue = new LinkedList<>();

    private boolean ballInProgress = false;
    private int inZoneFrames = 0;
    private boolean ballPresent = false;
    private boolean colorLocked = false;
    private Integer currentBallColor = null;

    private int stableFrames = 0;
    private static final int LOCK_FRAMES = 3;


    /* ===================== 构造 ===================== */

    public BallStorage(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class,"intakeDistanceSensor");
    }

    /* ===================== 主更新 ===================== */

    public void update() {

        double distance = distanceSensor.getDistance(DistanceUnit.CM);

        boolean inEnterZone = distance <= 3.6;
        boolean outExitZone = distance >= 5;

        // 读取颜色
        lastR = colorSensor.red();
        lastG = colorSensor.green();
        lastB = colorSensor.blue();
        Color.RGBToHSV(lastR, lastG, lastB, hsv);

        float H = hsv[0];
        float S = hsv[1];
        float V = hsv[2];

        boolean isGreen =
                H >= GREEN_MIN_H && H <= GREEN_MAX_H &&
                        S >= GREEN_MIN_S && V >= GREEN_MIN_V;

        boolean isPurple =
                H >= PURPLE_MIN_H && H <= PURPLE_MAX_H &&
                        S >= PURPLE_MIN_S;

        /* ---------- 球进入 ---------- */

        if (inEnterZone && !ballPresent) {
            ballPresent = true;
            colorLocked = false;
            currentBallColor = null;
            stableFrames = 0;
        }

        /* ---------- 颜色锁定（球仍在） ---------- */

        if (ballPresent && !colorLocked) {

            if (isGreen || isPurple) {
                stableFrames++;
            } else {
                stableFrames = 0; // 颜色不稳定直接清
            }

            if (stableFrames >= LOCK_FRAMES) {
                colorLocked = true;
                currentBallColor = isPurple ? 1 : 0;

                // ⚠️ 关键：在“球还没离开”时就入队
                colorQueue.offer(currentBallColor);
                while (colorQueue.size() > MAX_BALLS) {
                    colorQueue.poll();
                }
            }
        }

        /* ---------- 球离开 ---------- */

        if (ballPresent && outExitZone) {
            ballPresent = false;
            colorLocked = false;
            currentBallColor = null;
            stableFrames = 0;
        }
    }

    /* ===================== 球结束处理 ===================== */

    private void finalizeBall() {

        if (hueSamples.isEmpty()) return;

        Collections.sort(hueSamples);
        float medianHue = hueSamples.get(hueSamples.size() / 2);

        // 判最终颜色
        if (medianHue >= PURPLE_MIN_H && medianHue <= PURPLE_MAX_H) {
            colorQueue.offer(1); // Purple
        } else if (medianHue >= GREEN_MIN_H && medianHue <= GREEN_MAX_H) {
            colorQueue.offer(0); // Green
        }

        // 限制容量
        while (colorQueue.size() > MAX_BALLS) {
            colorQueue.poll();
        }
    }

    /* ===================== 对外接口 ===================== */

    public int getBallCount() {
        return colorQueue.size();
    }

    public boolean isFull() {
        return colorQueue.size() >= MAX_BALLS;
    }

    /** 0 = Green, 1 = Purple */
    public Queue<Integer> getColorQueue() {
        return colorQueue;
    }

    public void reset() {
        colorQueue.clear();
        hueSamples.clear();
        inZoneFrames = 0;
        ballInProgress = false;
    }

    /* ===================== Telemetry 调试 ===================== */

    public int getR() { return lastR; }
    public int getG() { return lastG; }

    public int getB() { return lastB; }

    public float getHue() { return hsv[0]; }
    public float getSaturation() { return hsv[1]; }
    public float getValue() { return hsv[2]; }

    public boolean isDetectingBall() {
        return ballInProgress;
    }
}
