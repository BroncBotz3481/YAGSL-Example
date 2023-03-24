package webblib.util;

public class Gains {
  public double P;
  public double I;
  public double D;
  public double F;
  public double integralZone;
  public double peakOutput;

  /**
   * Create a new gains profile for PID.
   *
   * @param P proportion
   * @param I integral
   * @param D derivative
   * @param F feed forward
   * @param integralZone izone
   * @param peakOutput peak motor output
   */
  public Gains(double P, double I, double D, double F, int integralZone, double peakOutput) {
    this.P = P;
    this.I = I;
    this.D = D;
    this.F = F;
    this.integralZone = integralZone;
    this.peakOutput = peakOutput;
  }
}
