package webblib.math;

import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

public class BetterMath {

  /**
   * Takes in an input in the domain (-inf, inf), puts it in a function defined only for a domain of
   * [0, inf) and a range of [0, inf), and returns a result of (-inf, inf). Essentially mirrors a
   * function defined in Quadrant I across the y-axis and x-axis so that it applies equally to
   * negative numbers but returns a negative number as a result. For example f(x) = x^2 would return
   * normal for x >=0, but negative on x < 0.
   *
   * @param input input of the function.
   * @param func function defined to return f(x) >= 0 for all x >= 0. Can be defined outside of x
   *     >=0 but this portion is not used.
   * @return signed output dependent on signage of input.
   */
  public static double signedAbsFunc(double input, DoubleFunction<Double> func) {
    var absOutput = func.apply(Math.abs(input));
    if (absOutput < 0)
      throw new IllegalArgumentException("func must be defined to return > 0 for all numbers > 0");
    return input >= 0 ? absOutput : -absOutput;
  }

  /**
   * Signage change for output of a function dependent on signage of a double. Commonly used for
   * something like magnitude calculation of a vector where the magnitude has to be signed.
   *
   * @param signage double where value doesn't matter, only signage does.
   * @param func function f() that returns an output > 0.
   * @return mirrored output across x and y axis dependent on signage double.
   */
  public static double signedAbsFunc(double signage, DoubleSupplier func) {
    var absOutput = func.getAsDouble();
    if (absOutput < 0)
      throw new IllegalArgumentException("func must be defined to return > 0 for all numbers > 0");
    return signage >= 0 ? absOutput : -absOutput;
  }
}
