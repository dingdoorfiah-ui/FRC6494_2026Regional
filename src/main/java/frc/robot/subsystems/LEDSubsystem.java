package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  public enum Mode {
    OFF,
    SOLID,
    BREATH,
    RAINBOW
  }

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final Timer timer = new Timer();

  // 左右灯带分段（左 0~71，右 72~143）
  private final int leftStart = 0;
  private final int leftLen;
  private final int rightStart;
  private final int rightLen;

  private Mode leftMode = Mode.OFF;
  private Mode rightMode = Mode.OFF;

  // 左右颜色
  private int lr = 0, lg = 0, lb = 0;
  private int rr = 0, rg = 0, rb = 0;

  // 动画参数
  private double leftBreathPeriodSec = 2.0;
  private double rightBreathPeriodSec = 2.0;
  private int leftRainbowOffset = 0;
  private int rightRainbowOffset = 0;

  /**
   * @param pwmPort  PWM 端口（只能用一个！）
   * @param leftCount 左灯数量
   * @param rightCount 右灯数量
   */
  public LEDSubsystem(int pwmPort, int leftCount, int rightCount) {
    this.leftLen = leftCount;
    this.rightLen = rightCount;
    this.rightStart = leftStart + leftLen;

    led = new AddressableLED(pwmPort);
    buffer = new AddressableLEDBuffer(leftLen + rightLen);

    led.setLength(buffer.getLength());
    led.start();

    timer.start();
  }

  /* ====================== */
  /*        对外API           */
  /* ====================== */

  public void offLeft() { leftMode = Mode.OFF; }
  public void offRight() { rightMode = Mode.OFF; }
  public void offAll() { leftMode = Mode.OFF; rightMode = Mode.OFF; }

  public void solidLeft(int r, int g, int b) {
    lr = clamp8(r); lg = clamp8(g); lb = clamp8(b);
    leftMode = Mode.SOLID;
  }

  public void solidRight(int r, int g, int b) {
    rr = clamp8(r); rg = clamp8(g); rb = clamp8(b);
    rightMode = Mode.SOLID;
  }

  public void breathLeft(int r, int g, int b, double periodSeconds) {
    lr = clamp8(r); lg = clamp8(g); lb = clamp8(b);
    leftBreathPeriodSec = Math.max(0.2, periodSeconds);
    leftMode = Mode.BREATH;
  }

  public void breathRight(int r, int g, int b, double periodSeconds) {
    rr = clamp8(r); rg = clamp8(g); rb = clamp8(b);
    rightBreathPeriodSec = Math.max(0.2, periodSeconds);
    rightMode = Mode.BREATH;
  }

  public void rainbowLeft() { leftMode = Mode.RAINBOW; }
  public void rainbowRight() { rightMode = Mode.RAINBOW; }
  public void rainbowAll() { leftMode = Mode.RAINBOW; rightMode = Mode.RAINBOW; }

  @Override
  public void periodic() {
    if(!Constants.LEDUsing) {led.close(); return;}
    // 左段
    switch (leftMode) {
      case OFF -> setRange(leftStart, leftLen, 0, 0, 0);
      case SOLID -> setRange(leftStart, leftLen, lr, lg, lb);
      case BREATH -> updateBreath(leftStart, leftLen, lr, lg, lb, leftBreathPeriodSec);
      case RAINBOW -> updateRainbow(leftStart, leftLen, true);
    }

    // 右段
    switch (rightMode) {
      case OFF -> setRange(rightStart, rightLen, 0, 0, 0);
      case SOLID -> setRange(rightStart, rightLen, rr, rg, rb);
      case BREATH -> updateBreath(rightStart, rightLen, rr, rg, rb, rightBreathPeriodSec);
      case RAINBOW -> updateRainbow(rightStart, rightLen, false);
    }

    led.setData(buffer);
  }

  /* ====================== */
  /*        内部实现          */
  /* ====================== */

  private void setRange(int start, int len, int r, int g, int b) {
    for (int i = 0; i < len; i++) {
      buffer.setRGB(start + i, r, g, b);
    }
  }

  private void updateBreath(int start, int len, int r, int g, int b, double periodSec) {
    double t = timer.get();
    double phase = (t % periodSec) / periodSec; // 0..1
    double brightness = 0.5 * (1.0 + Math.sin(2.0 * Math.PI * phase)); // 0..1
    int rr = (int) (r * brightness);
    int gg = (int) (g * brightness);
    int bb = (int) (b * brightness);
    setRange(start, len, rr, gg, bb);
  }

  private void updateRainbow(int start, int len, boolean isLeft) {
    int offset = isLeft ? leftRainbowOffset : rightRainbowOffset;

    for (int i = 0; i < len; i++) {
      int hue = (offset + (i * 180 / len)) % 180;
      buffer.setHSV(start + i, hue, 255, 128);
    }

    if (isLeft) leftRainbowOffset = (leftRainbowOffset + 2) % 180;
    else rightRainbowOffset = (rightRainbowOffset + 2) % 180;
  }

  private static int clamp8(int v) {
    return Math.max(0, Math.min(255, v));
  }
}
