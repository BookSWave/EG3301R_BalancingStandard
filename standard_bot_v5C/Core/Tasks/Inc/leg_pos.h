/*
 * leg_pos.h
 *
 *  Created on: Jul 13, 2024
 *      Author: YI MING
 */

#ifndef TASKS_INC_LEG_POS_H_
#define TASKS_INC_LEG_POS_H_
void leg_pos(double phi1, double phi4, double pos[2]);
void leg_spd(double dphi1, double dphi4, double phi1, double phi4,
             double spd[2]);
void leg_conv(double F, double Tp, double phi1, double phi4, double T[2]);


#endif /* TASKS_INC_LEG_POS_H_ */
