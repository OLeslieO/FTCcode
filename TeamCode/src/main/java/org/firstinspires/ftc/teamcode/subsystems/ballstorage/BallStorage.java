package org.firstinspires.ftc.teamcode.subsystems.ballstorage;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class BallStorage {

    /* ===================== 硬件 ===================== */

    private final RevColorSensorV3 colorSensor;

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

    /* ===================== 构造 ===================== */

    public BallStorage(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");
    }

    /* ===================== 主更新 ===================== */

    public void update() {

        // 读取 RGB
        lastR = colorSensor.red();
        lastG = colorSensor.green();
        lastB = colorSensor.blue();

        // 转 HSV
        Color.RGBToHSV(lastR, lastG, lastB, hsv);

        float H = hsv[0];
        float S = hsv[1];
        float V = hsv[2];

        // HSV 判定
        boolean isGreen =
                H >= GREEN_MIN_H && H <= GREEN_MAX_H &&
                        S >= GREEN_MIN_S && V >= GREEN_MIN_V;

        boolean isPurple =
                H >= PURPLE_MIN_H && H <= PURPLE_MAX_H &&
                        S >= PURPLE_MIN_S;

        boolean isBallInZone = isGreen || isPurple;

        /* ---------- 抗抖逻辑 ---------- */

        if (isBallInZone) {
            inZoneFrames++;

            if (inZoneFrames >= MIN_IN_ZONE_FRAMES) {
                hueSamples.add(H);
                ballInProgress = true;
            }

        } else {
            // 离开 zone，且之前确实有球
            if (ballInProgress) {
                finalizeBall();
            }

            inZoneFrames = 0;
            ballInProgress = false;
            hueSamples.clear();
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
