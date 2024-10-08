#include "leg_pos.h"
#include "leg_speed.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
/*
 * Arguments    : double u0
 *                double u1
 * Return Type  : double
 */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = atan2(i, i1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }
  return y;
}

/*
 * LEG_POS
 *     POS = LEG_POS(PHI1,PHI4)
 *
 * Arguments    : double phi1
 *                double phi4
 *                double pos[2]
 * Return Type  : void
 */

void leg_pos(double phi1, double phi4, double pos[2])
{
  double a;
  double b_a;
  double t14;
  double t15;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t8;
  /*     This function was generated by the Symbolic Math Toolbox version 24.1.
   */
  /*     20-Jul-2024 15:20:14 */
  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t6 = t2 * 0.08;
  t8 = t4 * 0.08;
  t14 = t4 * 0.024;
  t15 = t5 * 0.024;
  t5 = t8 - t5 * 0.08;
  a = t14 - t15;
  b_a = (t3 * 0.08 - t6) + 0.0983;
  t4 = (t3 * 0.024 - t2 * 0.024) + 0.02949;
  t5 = t5 * t5 + b_a * b_a;
  t4 = atan(1.0 / (t4 + t5) *
            ((t15 - t14) + sqrt((a * a + t4 * t4) - t5 * t5))) *
       2.0;
  t5 = t8 + sin(t4) * 0.15;
  t4 = (t6 + cos(t4) * 0.15) - 0.04915;
  pos[0] = sqrt(t5 * t5 + t4 * t4);
  pos[1] = rt_atan2d_snf(t5, t4);
}


void leg_conv(double F, double Tp, double phi1, double phi4, double T[2])
{
  double t100;
  double t102_tmp;
  double t108_tmp;
  double t120_tmp;
  double t123_tmp;
  double t124_tmp;
  double t131_tmp;
  double t16_tmp;
  double t174;
  double t175;
  double t18_tmp;
  double t21_tmp;
  double t22_tmp;
  double t24_tmp;
  double t25_tmp;
  double t50_tmp;
  double t5_tmp;
  double t60_tmp;
  double t65_tmp;
  double t77_tmp;
  double t7_tmp;
  double t81_tmp;
  double t82_tmp;
  double t91_tmp;
  /*     This function was generated by the Symbolic Math Toolbox version 24.1.
   */
  /*     20-Jul-2024 15:20:16 */
  t174 = cos(phi1);
  t5_tmp = cos(phi4);
  t175 = sin(phi1);
  t7_tmp = sin(phi4);
  t16_tmp = t174 * 0.08;
  t18_tmp = t175 * 0.08;
  t21_tmp = t174 * 0.024;
  t22_tmp = t5_tmp * 0.024;
  t24_tmp = t175 * 0.024;
  t25_tmp = t7_tmp * 0.024;
  t100 = t18_tmp - t7_tmp * 0.08;
  t50_tmp = t24_tmp - t25_tmp;
  t60_tmp = (t5_tmp * 0.08 - t16_tmp) + 0.0983;
  t65_tmp = (t22_tmp - t21_tmp) + 0.02949;
  t77_tmp = t100 * t100 + t60_tmp * t60_tmp;
  t81_tmp = t174 * t100 * 0.16 + t175 * t60_tmp * 0.16;
  t82_tmp = t5_tmp * t100 * 0.16 + t7_tmp * t60_tmp * 0.16;
  t60_tmp = 1.0 / (t65_tmp + t77_tmp);
  t91_tmp = t60_tmp * t60_tmp;
  t100 = sqrt((t50_tmp * t50_tmp + t65_tmp * t65_tmp) - t77_tmp * t77_tmp);
  t102_tmp = 1.0 / t100;
  t108_tmp = (t25_tmp - t24_tmp) + t100;
  t120_tmp = atan(t60_tmp * t108_tmp) * 2.0;
  t123_tmp = cos(t120_tmp);
  t124_tmp = sin(t120_tmp);
  t131_tmp = 1.0 / (t91_tmp * (t108_tmp * t108_tmp) + 1.0);
  t120_tmp = (t24_tmp + t81_tmp) * t91_tmp * t108_tmp +
             t60_tmp * (t21_tmp -
                        t102_tmp *
                            ((t174 * t50_tmp * 0.048 + t175 * t65_tmp * 0.048) -
                             t77_tmp * t81_tmp * 2.0) /
                            2.0);
  t21_tmp =
      (t25_tmp + t82_tmp) * t91_tmp * t108_tmp +
      t60_tmp * (t22_tmp -
                 t102_tmp *
                     ((t5_tmp * t50_tmp * 0.048 + t7_tmp * t65_tmp * 0.048) -
                      t77_tmp * t82_tmp * 2.0) /
                     2.0);
  t24_tmp = t18_tmp + t124_tmp * 0.15;
  t81_tmp = (t16_tmp + t123_tmp * 0.15) - 0.04915;
  t100 = (-t16_tmp - t123_tmp * 0.15) + 0.04915;
  t174 = t100 * t100;
  t175 = 1.0 / t100;
  t100 = t24_tmp * t24_tmp;
  t102_tmp = t123_tmp * t131_tmp;
  t91_tmp = t16_tmp - t102_tmp * t120_tmp * 0.3;
  t82_tmp = t124_tmp * t131_tmp;
  t120_tmp = t18_tmp - t82_tmp * t120_tmp * 0.3;
  t108_tmp = F * (1.0 / sqrt(t100 + t81_tmp * t81_tmp));
  t60_tmp = Tp * t174 * (1.0 / (t100 + t174));
  t100 = t24_tmp * (1.0 / t174);
  T[0] = t108_tmp * (t24_tmp * t91_tmp * 2.0 - t81_tmp * t120_tmp * 2.0) / 2.0 -
         t60_tmp * (t175 * t91_tmp - t100 * t120_tmp);
  T[1] = t108_tmp *
             (t102_tmp * t24_tmp * t21_tmp * 0.6 -
              t82_tmp * t81_tmp * t21_tmp * 0.6) /
             2.0 +
         t60_tmp * (t175 * (0.0 - t102_tmp * t21_tmp * 0.3) +
                    t100 * (t82_tmp * t21_tmp * 0.3));
}
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: leg_spd.c
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 16-Jul-2024 19:02:03
 */

/* Include Files */

/* Function Definitions */
/*
 * LEG_SPD
 *     SPD = LEG_SPD(DPHI1,DPHI4,PHI1,PHI4)
 *
 * Arguments    : double dphi1
 *                double dphi4
 *                double phi1
 *                double phi4
 *                double spd[2]
 * Return Type  : void
 */
void leg_spd(double dphi1, double dphi4, double phi1, double phi4,
             double spd[2])
{
  double t10_tmp;
  double t12_tmp;
  double t15;
  double t16;
  double t18;
  double t19;
  double t2;
  double t28;
  double t3;
  double t30;
  double t36;
  double t38;
  double t4;
  double t44;
  double t47;
  double t48;
  double t5;
  double t52;
  double t53;
  double t59;
  double t60;
  double t70;
  double t71;
  double t76;
  /*     This function was generated by the Symbolic Math Toolbox version 24.1.
   */
  /*     20-Jul-2024 15:20:15 */
  t2 = cos(phi1);
  t3 = cos(phi4);
  t4 = sin(phi1);
  t5 = sin(phi4);
  t10_tmp = t2 * 0.08;
  t12_tmp = t4 * 0.08;
  t15 = t2 * 0.024;
  t16 = t3 * 0.024;
  t18 = t4 * 0.024;
  t19 = t5 * 0.024;
  t28 = t12_tmp - t5 * 0.08;
  t30 = t18 - t19;
  t36 = (t3 * 0.08 - t10_tmp) + 0.0983;
  t38 = (t16 - t15) + 0.02949;
  t44 = t28 * t28 + t36 * t36;
  t47 = t2 * t28 * 0.16 + t4 * t36 * 0.16;
  t48 = t3 * t28 * 0.16 + t5 * t36 * 0.16;
  t52 = 1.0 / (t38 + t44);
  t53 = t52 * t52;
  t59 = sqrt((t30 * t30 + t38 * t38) - t44 * t44);
  t60 = 1.0 / t59;
  t36 = (t19 - t18) + t59;
  t28 = atan(t52 * t36) * 2.0;
  t70 = cos(t28);
  t71 = sin(t28);
  t76 = 1.0 / (t53 * (t36 * t36) + 1.0);
  t47 = (t18 + t47) * t53 * t36 +
        t52 * (t15 -
               t60 * ((t2 * t30 * 0.048 + t4 * t38 * 0.048) - t44 * t47 * 2.0) /
                   2.0);
  t28 = (t19 + t48) * t53 * t36 +
        t52 * (t16 -
               t60 * ((t3 * t30 * 0.048 + t5 * t38 * 0.048) - t44 * t48 * 2.0) /
                   2.0);
  t4 = t12_tmp + t71 * 0.15;
  t15 = (t10_tmp + t70 * 0.15) - 0.04915;
  t59 = t70 * t76;
  t18 = t59 * t28;
  t53 = t71 * t76;
  t2 = t53 * t28;
  t28 = (-t10_tmp - t70 * 0.15) + 0.04915;
  t60 = t28 * t28;
  t52 = 1.0 / t28;
  t36 = t4 * t4;
  t28 = 1.0 / sqrt(t36 + t15 * t15);
  t48 = 1.0 / (t36 + t60);
  t59 = t10_tmp - t59 * t47 * 0.3;
  t36 = t12_tmp - t53 * t47 * 0.3;
  spd[0] = dphi4 * t28 * (t4 * t18 * 0.6 - t15 * t2 * 0.6) / 2.0 +
           dphi1 * t28 * (t4 * t59 * 2.0 - t15 * t36 * 2.0) / 2.0;
  t28 = t4 * (1.0 / t60);
  spd[1] = -dphi1 * t60 * t48 * (t52 * t59 - t28 * t36) +
           dphi4 * t60 * t48 * (t52 * (0.0 - t18 * 0.3) + t28 * (t2 * 0.3));
}

/*
 * File trailer for leg_spd.c
 *
 * [EOF]
 */
