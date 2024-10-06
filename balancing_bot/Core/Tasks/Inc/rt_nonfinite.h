/*
 * rt_nonfinite.h
 *
 *  Created on: Jul 17, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_RT_NONFINITE_H_
#define TASKS_INC_RT_NONFINITE_H_
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 24.1
 * C/C++ source code generated on  : 17-Jul-2024 13:16:56
 */

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H

/* Include Files */
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */



#endif /* TASKS_INC_RT_NONFINITE_H_ */
