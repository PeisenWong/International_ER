//#include "PP.h"
#include "../adapter.h"
#include <math.h>

void PPInit  (uint8_t base,float *qeix, float *qeiy, float*imu,PathPlan_t *pp, Laser_t *lsrx, Laser_t *lsry, Laser_t* lsrR){

	pp->base_shape=base;
	pp->qeix = qeix;
	pp->qeiy = qeiy;
	pp->lsrx = &(lsrx->dist);
	pp->lsry = &(lsry->dist);
	pp->lsrR = &(lsrR->dist);
	pp->lsrx_swap=0;
	pp->lsry_swap=0;
	pp->yaw = imu;
	pp->point_start=1;

	PIDSourceInit(&(pp->error_x), &(pp->outx), &(pp->x));
	PIDGainInit(0.005, 1.0, 1.0, 1.0, 1.0, 0.0, 0.4, 30.0, &(pp->x));
	PIDDelayInit(&(pp->x));

	PIDSourceInit(&(pp->error_y), &(pp->outy), &(pp->y));
	PIDGainInit(0.005, 1.0, 1.0, 1.0, 1.0, 0.0, 0.4, 30.0, &(pp->y));
	PIDDelayInit(&(pp->y));

	PIDSourceInit(&(pp->error_z), &(pp->outz), &(pp->z));
	PIDGainInit(0.005, 1.0, (1.0 / 30.0), 0.5, 5.0, 0.2, 0.2, 30.0, &(pp->z));
	PIDDelayInit(&(pp->z));

	pp->tol_xy_crv= 0.6;

	pp->yaw_offset=*(pp->yaw);
}

void PP_PIDPathSet(float kp, float ki, float kd, PathPlan_t *pp){

	pp->kp[0]=kp;
	pp->ki[0]=ki;
	pp->kd[0]=kd;

	PIDGainSet(KP,kp,&(pp->x));
	PIDGainSet(KI,ki,&(pp->x));
	PIDGainSet(KD,kd,&(pp->x));

	PIDGainSet(KP,kp,&(pp->y));
	PIDGainSet(KI,ki,&(pp->y));
	PIDGainSet(KD,kd,&(pp->y));
}

void PP_PIDZSet(float kp, float ki, float kd, float ku, PathPlan_t *pp){

	PIDGainSet(KP,kp,&(pp->z));
	PIDGainSet(KI,ki,&(pp->z));
	PIDGainSet(KD,kd,&(pp->z));
	PIDGainSet(KU,ku,&(pp->z));
	pp->pp_orgz_kp = kp;

}
void PP_PIDEndSet(float kp, float ki, float kd, PathPlan_t *pp){

	pp->kp[1]=kp;
	pp->ki[1]=ki;
	pp->kd[1]=kd;

	PIDGainSet(KP,kp,&(pp->x));
	PIDGainSet(KI,ki,&(pp->x));
	PIDGainSet(KD,kd,&(pp->x));

	PIDGainSet(KP,kp,&(pp->y));
	PIDGainSet(KI,ki,&(pp->y));
	PIDGainSet(KD,kd,&(pp->y));
}

void PP_start(float point[][7],int no_point,PathPlan_t *pp){

	int i;
	for(i=0;i<no_point;i++){
		pp->target_vel[i] = point[i][0];
		pp->target_x[i] = point[i][1];
		pp->target_y[i] = point[i][2];
		pp->target_accurate[i] = point[i][5];
		pp->pp_crv_radius[i] =  point[i][6];
		if(i == 0)
			pp->target_angle1[0] = atanf((fabs)(point[0][2] - pp->real_y) / (fabs)(point[0][1]- pp->real_x));
		else
			pp->target_angle1[i]= atanf((fabs)(point[i][2]-point[i-1][2]) / (fabs)(point[i][1]-point[i-1][1]));
		pp->target_z[i] = point[i][3];
		pp->ku_x[i] = point[i][4]* cosf(pp->target_angle1[i]);
		pp->ku_y[i] = point[i][4]* sinf(pp->target_angle1[i]);
	}

	pp->target_point=no_point;
	if(fabs(pp->real_x - pp->target_x[0])){
		PIDGainSet(KE,1.0/fabs(pp->real_x - pp->target_x[0]),&(pp->x));
	} else {
		PIDGainSet(KE,1.0,&(pp->x));
	}
	if(fabs(pp->real_y - pp->target_y[0])){
		PIDGainSet(KE,1.0/fabs(pp->real_y - pp->target_y[0]),&(pp->y));
	} else {
		PIDGainSet(KE,1.0,&(pp->y));
	}

	PIDGainSet(KU,pp->ku_x[0],&(pp->x));
	PIDGainSet(KU,pp->ku_y[0],&(pp->y));

	pp->point_count=0;
	pp->crnt_crv_pt=0;
	pp->pp_crv_calc=0;
	pp->pp_start=1;
	pp->rotate=0;

	pp->tol_xy=0.20;
	pp->tol_z=2.0;

	pp->f_tol_xy[0]=0.06;
	pp->f_tol_z[0]=1.0;
//
//		sprintf(uartbuff,"%f %f %f %f %f %f %f\r\n",point[0][0],point[0][1],
//				point[0][2],point[0][3],point[0][4],
//				point[0][5],point[0][6]);
//							UARTPrintString(UART5,uartbuff);

}

void LSR_start(float point[][7], int no_point, PathPlan_t *pp, uint8_t single, uint8_t right){

	int i;
	for(i=0;i<no_point;i++){
		pp->target_vel[i] = point[i][0];
		pp->target_x[i] = point[i][1];
		pp->target_y[i] = point[i][2];
		pp->target_z[i] = point[i][3];
		pp->ku_x[i] = point[i][4];
		pp->ku_y[i] = point[i][4];
		pp->f_tol_xy[i]= point[i][5];
		pp->f_tol_z[i]= point[i][6];

	}

	PIDGainSet(KU,pp->ku_x[0],&(pp->x));
	PIDGainSet(KU, 1.5, &(pp->y));
	PP_PIDEndSet(1.0, 1.5, 0.8, pp);

	if(right)
	{
		pp->error_x = -(pp->target_x[pp->point_count] - *(pp->lsrR));
		if(fabs(pp->error_x)){
			PIDGainSet(KE,1.0/fabs(pp->error_x),&(pp->x));
		} else {
			PIDGainSet(KE,1.0,&(pp->x));
		}
		pp->right_lsr = 1;
	}
	else
	{
		pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
		if(fabs(pp->error_x)){
			PIDGainSet(KE,1.0/fabs(pp->error_x),&(pp->x));
		} else {
			PIDGainSet(KE,1.0,&(pp->x));
		}

		pp->right_lsr = 0;
	}

	pp->target_point=no_point;

	pp->lsr_start=1;
	pp->point_count=0;
	pp->lsr_init=0;
	pp->pp_lck_enb = 0;
	pp->pp_lck_count = 0.0;
	pp->pp_lck_cal   = 0.0;

	pp->rotate=0;

	PIDDelayInit(&(pp->x));
	PIDDelayInit(&(pp->y));
	PIDDelayInit(&(pp->z));

}

void PP_stop (PathPlan_t *pp){

	pp->pp_start=0;
	pp->lsr_start=0;
	PIDDelayInit(&(pp->x));
	PIDDelayInit(&(pp->y));
	PIDDelayInit(&(pp->z));


//	LED4 = 1;

	pp->pp_lck_enb = 0;
}

void PP_reset (PathPlan_t *pp){
	pp->pos_x = 0.0;
	pp->pos_y = 0.0;
	pp->prev_x = 0.0;
	pp->prev_y = 0.0;
	pp->prev_real_x = 0.0;
	pp->prev_real_y = 0.0;
	pp->del_pos_x = 0.0;
	pp->del_pos_y = 0.0;
	//	pp->yaw_constant = 0.0;
	//	pp->yaw_offset = *(pp->yaw);
	//	pp->prev_yaw = *(pp->yaw);
	pp->real_x=0.0;
	pp->real_y=0.0;
	PIDDelayInit(&(pp->x));
	PIDDelayInit(&(pp->y));
	PIDDelayInit(&(pp->z));
	//	pp->real_z=0.0;
}


void PP_SetZ (float z,PathPlan_t *pp){

	pp->setz=z;
	pp->yaw_constant=0;
	//	pp->yaw_offset=*(pp->yaw);
	pp->prev_yaw=*(pp->yaw);

}

void PP_SetCrv_Points (int z,PathPlan_t *pp){

	pp->pp_no_crv_pts = z;

}

void PathPlan (PathPlan_t *pp){

	if(pp->point_start){
		pp->pos_x=*(pp->qeix);
		pp->pos_y=*(pp->qeiy);

		if(*(pp->yaw) < 50.0){
			if(pp->prev_yaw > 330.0){
				pp->yaw_constant++;
			}
		}else if(*(pp->yaw) > 330.0){
			if(pp->prev_yaw < 50.0){
				pp->yaw_constant--;
			}
		}


		pp->prev_yaw = *(pp->yaw);
		pp->real_z = *(pp->yaw) + (pp->yaw_constant)*360.0 - pp->yaw_offset+pp->setz;
		pp->real_z_rad = (pp->real_z / 180.0) * 3.141593;

		pp->del_pos_x = pp->pos_x - pp->prev_x;
		pp->del_pos_y = pp->pos_y - pp->prev_y;

		pp->del_pos_x =  (pp->pos_x - pp->prev_x) * cosf(pp->real_z_rad) +
				(pp->pos_y - pp->prev_y) * sinf(pp->real_z_rad);
		pp->del_pos_y = -(pp->pos_x - pp->prev_x) * sinf(pp->real_z_rad) +
				(pp->pos_y - pp->prev_y) * cosf(pp->real_z_rad);

		pp->real_x = pp->real_x + pp->del_pos_x;
		pp->real_y = pp->real_y + pp->del_pos_y;

		pp->prev_x = pp->pos_x;
		pp->prev_y = pp->pos_y;

		pp->prev_real_x = pp->real_x;
		pp->prev_real_y = pp->real_y;
		pp->prev_real_z= pp->real_z;
	}


	if(pp->point_start && pp->pp_start){ // If path plan started

		if(pp->pp_crv_radius[pp->point_count]){


			if(!pp->pp_crv_calc){
				pp->pp_rad_ptx = ((pp->target_x[pp->point_count] - pp->real_x)/(float)2.0) + (pp->pp_crv_radius[pp->point_count] * sinf(atan2f(pp->target_y[pp->point_count] - pp->real_y, pp->target_x[pp->point_count] - pp->real_x)));
				pp->pp_rad_pty = ((pp->target_y[pp->point_count] - pp->real_y)/(float)2.0) - (pp->pp_crv_radius[pp->point_count] * cosf(atan2f(pp->target_y[pp->point_count] - pp->real_y, pp->target_x[pp->point_count] - pp->real_x)));
				pp->pp_crv_const[pp->point_count] = pp->target_z[pp->point_count]/pp->pp_no_crv_pts;

				float crvpath = (float)1.0/pp->pp_no_crv_pts;
				for(int i = 0;i<pp->pp_no_crv_pts;i++){
					pp->pp_crv_x[i]= powf((1-crvpath),2) * pp->real_x + 2.0*(1-crvpath)*crvpath*pp->pp_rad_ptx + powf(crvpath,2) * pp->target_x[pp->point_count];
					pp->pp_crv_y[i]= powf((1-crvpath),2) * pp->real_y + 2.0*(1-crvpath)*crvpath*pp->pp_rad_pty + powf(crvpath,2) * pp->target_y[pp->point_count];
					pp->pp_crv_z[i] = pp->pp_crv_const[pp->point_count] + pp->pp_crv_const[pp->point_count]*i;
					crvpath+=(float)1.0/pp->pp_no_crv_pts;
				}
				//				PIDGainSet(KP,(pp->target_vel[pp->point_count]/(float)4.0),&(pp->z));
				pp->pp_crv_calc = 1;
			}

			pp->error_x = pp->pp_crv_x[pp->crnt_crv_pt] - pp->real_x;
			pp->error_y = pp->pp_crv_y[pp->crnt_crv_pt] - pp->real_y;
			pp->error_z = pp->pp_crv_z[pp->crnt_crv_pt] - pp->real_z;


			if( pp->crnt_crv_pt == pp->pp_no_crv_pts - 1 && ((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z)
				pp->pp_lck = 1.0;
			else
				pp->pp_lck = 0.0;

			pp->pp_lck_count = pp->pp_lck_count + 1.0;

			if(pp->pp_lck_count <= 60.0)
				pp->pp_lck_cal = pp->pp_lck_cal + pp->pp_lck;
			else{
				pp->pp_lck_final = (pp->pp_lck_cal) / (pp->pp_lck_count) ;
				if(pp->pp_lck_final >= 0.95)
					pp->pp_lck_enb = 1;
				else
					pp->pp_lck_enb = 0;
				pp->pp_lck_count = 0.0;
				pp->pp_lck_cal   = 0.0;
			}


			if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy_crv){
				if(pp->crnt_crv_pt < pp->pp_no_crv_pts - 1){
					PIDGainSet(KP,(pp->pp_orgz_kp),&(pp->z));
					pp->crnt_crv_pt++;
					pp->error_x = pp->pp_crv_x[pp->crnt_crv_pt] - pp->real_x;
					pp->error_y = pp->pp_crv_y[pp->crnt_crv_pt] - pp->real_y;
					pp->error_z = pp->pp_crv_z[pp->crnt_crv_pt] - pp->real_z;

					if(pp->target_x[pp->point_count]-pp->real_x){
						PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
					} else {
						PIDGainSet(KE,1.0,&(pp->x));
					}
					if(pp->target_y[pp->point_count]-pp->real_y){
						PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
					} else {
						PIDGainSet(KE,1.0,&(pp->y));
					}
					PIDGainSet(KP,pp->kp[0],&(pp->x));
					PIDGainSet(KP,pp->kp[0],&(pp->y));
					PIDGainSet(KI,pp->ki[0],&(pp->x));
					PIDGainSet(KI,pp->ki[0],&(pp->y));
					PIDGainSet(KD,pp->kd[0],&(pp->x));
					PIDGainSet(KD,pp->kd[0],&(pp->y));
					PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
					PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));


				}else if(pp->crnt_crv_pt == pp->pp_no_crv_pts - 1 && (pp->point_count < (pp->target_point - 1)) ){
					if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z){
						if(pp->target_accurate[pp->point_count] == 1.0){
							if(pp->pp_lck_enb == 1){

								pp->point_count++;
								if(pp->target_x[pp->point_count]-pp->real_x){
									PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
								} else {
									PIDGainSet(KE,1.0,&(pp->x));
								}
								if(pp->target_y[pp->point_count]-pp->real_y){
									PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
								} else {
									PIDGainSet(KE,1.0,&(pp->y));
								}
								PIDGainSet(KP,pp->kp[0],&(pp->x));
								PIDGainSet(KP,pp->kp[0],&(pp->y));
								PIDGainSet(KI,pp->ki[0],&(pp->x));
								PIDGainSet(KI,pp->ki[0],&(pp->y));
								PIDGainSet(KD,pp->kd[0],&(pp->x));
								PIDGainSet(KD,pp->kd[0],&(pp->y));
								PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
								PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
								pp->crnt_crv_pt=0;
								pp->pp_crv_calc = 0;

								pp->pp_lck_enb = 0;

							}
						}else{
							pp->point_count++;
							if(pp->target_x[pp->point_count]-pp->real_x){
								PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
							} else {
								PIDGainSet(KE,1.0,&(pp->x));
							}
							if(pp->target_y[pp->point_count]-pp->real_y){
								PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
							} else {
								PIDGainSet(KE,1.0,&(pp->y));
							}
							PIDGainSet(KP,pp->kp[0],&(pp->x));
							PIDGainSet(KP,pp->kp[0],&(pp->y));
							PIDGainSet(KI,pp->ki[0],&(pp->x));
							PIDGainSet(KI,pp->ki[0],&(pp->y));
							PIDGainSet(KD,pp->kd[0],&(pp->x));
							PIDGainSet(KD,pp->kd[0],&(pp->y));
							PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
							PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
							pp->crnt_crv_pt=0;
							pp->pp_crv_calc = 0;
						}
					}

				}else if(pp->crnt_crv_pt == pp->pp_no_crv_pts - 1 && pp->point_count == (pp->target_point - 1) ){

					if(fabs(pp->error_x)< pp->f_tol_xy[0] && fabs(pp->error_y)<pp->f_tol_xy[0] && (int)pp->error_z<=pp->f_tol_z[0]){

						if(pp->target_accurate[pp->point_count] == 1.0){
							if(pp->pp_lck_enb == 1){
								PIDGainSet(KP,(pp->pp_orgz_kp),&(pp->z));
								//								APPStop();
//								RNSStop(&rns);
//								PP_stop(&pp);
								HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
								//								LED4 = 1;
								PIDDelayInit(&(pp->x));
								PIDDelayInit(&(pp->y));
								PIDDelayInit(&(pp->z));
								pp->crnt_crv_pt=0;
								pp->pp_crv_calc = 0;
								pp->pp_start=0;
								pp->pp_lck_enb = 0;
							}
						}else{
							PIDGainSet(KP,(pp->pp_orgz_kp),&(pp->z));
//							RNSStop(&rns);
//							PP_stop(&pp);
							//							APPStop();
							//							LED4 = 1;
							HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
							PIDDelayInit(&(pp->x));
							PIDDelayInit(&(pp->y));
							PIDDelayInit(&(pp->z));
							pp->crnt_crv_pt=0;
							pp->pp_crv_calc = 0;
							pp->pp_start=0;
						}
					}

				}
			}


			if(pp->pp_start){

				pp->dx = pp->pp_crv_x[pp->crnt_crv_pt] - pp->prev_real_x;
				pp->dy = pp->pp_crv_y[pp->crnt_crv_pt] - pp->prev_real_y;

				pp->rotate=0;

				if ((pp->dx != 0.0 || pp->dx != -0.0)&&(pp->dy != -0.0 || pp->dy != 0.0)){
					pp->heading = atan2f(pp->dy, pp->dx);
				} else {
					if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy < 0.0) {
						pp->heading = 1.5708;
					} else if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy > 0.0) {
						pp->heading = -1.5708;
					} else {
						pp->heading = 0.0;
						pp->rotate = 1;
					}
				}

				pp->vx  = pp->target_vel[pp->point_count] * cosf(pp->heading);
				pp->vy  = pp->target_vel[pp->point_count] * sinf(pp->heading);

				if(pp->rotate){
					pp->vx = pp->vy = 0.0;
				}

				PID(&(pp->x));
				PID(&(pp->y));
				PID(&(pp->z));

				pp->rux =   pp->outx*cosf(pp->real_z_rad) - pp->outy*sinf(pp->real_z_rad);
				pp->ruy =   pp->outx*sinf(pp->real_z_rad) + pp->outy*cosf(pp->real_z_rad);
				pp->rvx =   pp->vx*cosf(pp->real_z_rad) - pp->vy*sinf(pp->real_z_rad);
				pp->rvy =   pp->vx*sinf(pp->real_z_rad) + pp->vy*cosf(pp->real_z_rad);

				if(pp->base_shape== 0){
					pp->u1 = 0.707107 * ( pp->ruy - pp->rux) - (pp->outz * 1.0);
					pp->u2 = 0.707107 * ( pp->ruy + pp->rux) + (pp->outz * 1.0);
					pp->u3 = 0.707107 * ( pp->ruy - pp->rux) + (pp->outz * 1.0);
					pp->u4 = 0.707107 * ( pp->ruy + pp->rux) - (pp->outz * 1.0);

					pp->v1 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u1;
					pp->v2 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u2;
					pp->v3 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u3;
					pp->v4 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u4;

					//				pp->move(pp->v2,pp->v1,pp->v3,pp->v4,pp->rns);
				} else if(pp->base_shape==1){

					pp->u1 = (0.866 * pp->ruy) - (0.5 * pp->rux) + (pp->outz * 1.0);
					pp->u2 = (0.866 * pp->ruy) + (0.5 * pp->rux) - (pp->outz * 1.0);
					pp->u3 = 1.0 * pp->rux + (pp->outz * 1.0);
					pp->v1 = (0.866 * pp->rvy) - (0.5 * pp->rvx) + pp->u1;
					pp->v2 = (0.866 * pp->rvy) + (0.5 * pp->rvx) + pp->u2;
					pp->v3 = 1.0 * pp->rvx + pp->u3;
					//				pp->move(pp->v2,pp->v1,pp->v3,0.0,pp->rns);
				}

			}


		}
		// No curve moving
		else{
			led3 = 1;
			pp->error_x = pp->target_x[pp->point_count] - pp->real_x;
			pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
			pp->error_z = pp->target_z[pp->point_count] - pp->real_z;

			// I think code here is to calculate time to enable pp_lck_enb or not
			// If error < tol, add pp_lck
			if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z)
				pp->pp_lck = 1.0;
			else
				pp->pp_lck = 0.0;

			// Increase when no moving curve
			pp->pp_lck_count = pp->pp_lck_count + 1.0;

			if(pp->pp_lck_count <= 60.0)
				pp->pp_lck_cal = pp->pp_lck_cal + pp->pp_lck;
			else{
				pp->pp_lck_final = (pp->pp_lck_cal) / (pp->pp_lck_count) ;
				if(pp->pp_lck_final >= 0.95)
					pp->pp_lck_enb = 1;
				else
					pp->pp_lck_enb = 0;
				pp->pp_lck_count = 0.0;
				pp->pp_lck_cal   = 0.0;
			}


			if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z){ // Already almost at destination
				if(pp->point_count < (pp->target_point - 2)){ // Checking is second last point or not
					if(pp->target_accurate[pp->point_count] == 1.0){ // point_lock enable
						if(pp->pp_lck_enb == 1){ // After < tol some time, lck_enb = 1
							pp->point_count++; // Showing reached previous destination
							pp->error_x = pp->target_x[pp->point_count] - pp->real_x;
							pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
							pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
							if(pp->target_x[pp->point_count]-pp->real_x){
								PIDGainInit(0.005,
										1.0,
										1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),
										pp->ku_x[pp->point_count],
										pp->kp[0],
										pp->ki[0],
										pp->kd[0],
										30.0,
										&(pp->x));
								//								PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
							} else {
								PIDGainInit(0.005,
										1.0,
										1.0,
										pp->ku_x[pp->point_count],
										pp->kp[0],
										pp->ki[0],
										pp->kd[0],
										30.0,
										&(pp->x));
								//								PIDGainSet(KE,1.0,&(pp->x));
							}
							if(pp->target_y[pp->point_count]-pp->real_y){
								PIDGainInit(0.005,
										1.0,
										1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
										pp->ku_y[pp->point_count],
										pp->kp[0],
										pp->ki[0],
										pp->kd[0],
										30.0,
										&(pp->y));
								//								PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
							} else {
								PIDGainInit(0.005,
										1.0,
										1.0,
										pp->ku_y[pp->point_count],
										pp->kp[0],
										pp->ki[0],
										pp->kd[0],
										30.0,
										&(pp->y));
								//								PIDGainSet(KE,1.0,&(pp->y));
							}

							//							PIDGainSet(KP,pp->kp[0],&(pp->x));
							//							PIDGainSet(KP,pp->kp[0],&(pp->y));
							//							PIDGainSet(KI,pp->ki[0],&(pp->x));
							//							PIDGainSet(KI,pp->ki[0],&(pp->y));
							//							PIDGainSet(KD,pp->kd[0],&(pp->x));
							//							PIDGainSet(KD,pp->kd[0],&(pp->y));
							//							PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
							//							PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
							pp->pp_lck_enb = 0;
						}
					}
					else{ // Already between tol, but point lck not enable
						pp->point_count++; // Assuming reached previous point
						pp->error_x = pp->target_x[pp->point_count] - pp->real_x;
						pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
						pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
						if(pp->target_x[pp->point_count]-pp->real_x){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),
//									pp->ku_x[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->x));
							PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
						} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_x[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->x));
							PIDGainSet(KE,1.0,&(pp->x));
						}
						if(pp->target_y[pp->point_count]-pp->real_y){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
//									pp->ku_y[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->y));
							PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
						} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_y[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->y));
							PIDGainSet(KE,1.0,&(pp->y));
						}
					}
				} else if(pp->point_count == (pp->target_point - 2)){ // Reached second last going last point
					if(pp->target_accurate[pp->point_count] == 1.0){ // point_lock enable
						if(pp->pp_lck_enb == 1){
							pp->point_count++;
							pp->error_x = pp->target_x[pp->point_count] - pp->real_x;
							pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
							pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
							if(pp->target_x[pp->point_count]-pp->real_x){
								PIDGainInit(0.005,
										1.0,
										1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),
										pp->ku_x[pp->point_count],
										pp->kp[1],
										pp->ki[1],
										pp->kd[1],
										30.0,
										&(pp->x));
								//								PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
							} else {
								PIDGainInit(0.005,
										1.0,
										1.0,
										pp->ku_x[pp->point_count],
										pp->kp[1],
										pp->ki[1],
										pp->kd[1],
										30.0,
										&(pp->x));
								//								PIDGainSet(KE,1.0,&(pp->x));
							}
							if(pp->target_y[pp->point_count]-pp->real_y){
								PIDGainInit(0.005,
										1.0,
										1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
										pp->ku_y[pp->point_count],
										pp->kp[1],
										pp->ki[1],
										pp->kd[1],
										30.0,
										&(pp->y));
								//								PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
							} else {
								PIDGainInit(0.005,
										1.0,
										1.0,
										pp->ku_y[pp->point_count],
										pp->kp[1],
										pp->ki[1],
										pp->kd[1],
										30.0,
										&(pp->y));
								//								PIDGainSet(KE,1.0,&(pp->y));
							}
							pp->pp_lck_enb = 0;
						}
					}
					else{
						pp->point_count++;
						pp->error_x = pp->target_x[pp->point_count] - pp->real_x;
						pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
						pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
						if(pp->target_x[pp->point_count]-pp->real_x){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),
//									pp->ku_x[pp->point_count],
//									pp->kp[1],
//									pp->ki[1],
//									pp->kd[1],
//									30.0,
//									&(pp->x));
							PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
						} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_x[pp->point_count],
//									pp->kp[1],
//									pp->ki[1],
//									pp->kd[1],
//									30.0,
//									&(pp->x));
							PIDGainSet(KE,1.0,&(pp->x));
						}
						if(pp->target_y[pp->point_count]-pp->real_y){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
//									pp->ku_y[pp->point_count],
//									pp->kp[1],
//									pp->ki[1],
//									pp->kd[1],
//									30.0,
//									&(pp->y));
							PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
						} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_y[pp->point_count],
//									pp->kp[1],
//									pp->ki[1],
//									pp->kd[1],
//									30.0,
//									&(pp->y));
							PIDGainSet(KE,1.0,&(pp->y));
						}
					}
				}else if(fabs(pp->error_x)<pp->f_tol_xy[0] && fabs(pp->error_y)<pp->f_tol_xy[0] && (int)pp->error_z<=pp->f_tol_z[0]){ // Reached last point
					if(pp->target_accurate[pp->point_count] == 1.0){
						if(pp->pp_lck_enb == 1){
//							RNSStop(&rns);
//							PP_stop(&pp);
							led3 = 0;
							//								APPStop();
							//								LED4 = 1;
							PIDDelayInit(&(pp->x));
							PIDDelayInit(&(pp->y));
							PIDDelayInit(&(pp->z));
							pp->pp_start=0;
							pp->pp_lck_enb = 0;
						}
					}
					else{
						//							APPStop();
						//							LED4 = 1;
						led3 = 0;
//						RNSStop(&rns);
//						PP_stop(&pp);
						PIDDelayInit(&(pp->x));
						PIDDelayInit(&(pp->y));
						PIDDelayInit(&(pp->z));
						pp->pp_start=0;
					}
				}
			}

			if(pp->pp_start){

				pp->dx = pp->target_x[pp->point_count] - pp->prev_real_x;
				pp->dy = pp->target_y[pp->point_count] - pp->prev_real_y;

				pp->rotate=0;

				if ((pp->dx != 0.0 || pp->dx != -0.0)&&(pp->dy != -0.0 || pp->dy != 0.0)){
					pp->heading = atan2f(pp->dy, pp->dx);
				} else {
					if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy < 0.0) { // Backward
						pp->heading = -1.5708;
					} else if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy > 0.0) { // Forward
						pp->heading = 1.5708;
					} else if((pp->dy == 0.0 || pp->dy == -0.0) && pp->dx > 0){ // Right
						pp->heading = 0.0;
//						pp->rotate = 1;
					}
					else if((pp->dy == 0.0 || pp->dy == -0.0) && pp->dx < 0) // Left
					{
						pp->heading = M_PI;
					}
				}

				pp->vx  = pp->target_vel[pp->point_count] * cosf(pp->heading);
				pp->vy  = pp->target_vel[pp->point_count] * sinf(pp->heading);

				if(pp->rotate){
					pp->vx = pp->vy = 0.0;
				}

				PID(&(pp->x));
				PID(&(pp->y));
				PID(&(pp->z));

				pp->rux =   pp->outx*cosf(pp->real_z_rad) - pp->outy*sinf(pp->real_z_rad);
				pp->ruy =   pp->outx*sinf(pp->real_z_rad) + pp->outy*cosf(pp->real_z_rad);
				pp->rvx =   pp->vx*cosf(pp->real_z_rad) - pp->vy*sinf(pp->real_z_rad);
				pp->rvy =   pp->vx*sinf(pp->real_z_rad) + pp->vy*cosf(pp->real_z_rad);

				if(pp->base_shape== fwd_omni){
//					pp->u1 = 0.707107 * ( pp->ruy + pp->rux) + (pp->outz * 1.0);
//					pp->u2 = 0.707107 * ( pp->ruy - pp->rux) - (pp->outz * 1.0);
//					pp->u3 = 0.707107 * ( pp->ruy - pp->rux) + (pp->outz * 1.0);
//					pp->u4 = 0.707107 * ( pp->ruy + pp->rux) - (pp->outz * 1.0);

					pp->u1 = (pp->outz * 1.0);
					pp->u2 = -pp->outz * 1.0;
					pp->u3 = (pp->outz * 1.0);
					pp->u4 = -(pp->outz * 1.0);


					pp->v1 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u1;
					pp->v2 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u2;
					pp->v3 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u3;
					pp->v4 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u4;

					//				pp->move(pp->v2,pp->v1,pp->v3,pp->v4,pp->rns);
				} else if(pp->base_shape== 1){

					pp->u1 = (0.866 * pp->ruy) - (0.5 * pp->rux) + (pp->outz * 1.0);
					pp->u2 = (0.866 * pp->ruy) + (0.5 * pp->rux) - (pp->outz * 1.0);
					pp->u3 = 1.0 * pp->rux + (pp->outz * 1.0);
					pp->v1 = (0.866 * pp->rvy) - (0.5 * pp->rvx) + pp->u1;
					pp->v2 = (0.866 * pp->rvy) + (0.5 * pp->rvx) + pp->u2;
					pp->v3 = 1.0 * pp->rvx + pp->u3;
					//				pp->move(pp->v2,pp->v1,pp->v3,0.0,pp->rns);
				}
//				hb_count = HAL_GetTick();

			}
		}
	}
	else if(pp->point_start && pp->lsr_start) // In this code, only moving in x direction but using left and right
	{
		led3 = 1;
		if(!pp->right_lsr)
			pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
		else
			pp->error_x = -(pp->target_x[pp->point_count] - *(pp->lsrR));
		pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
		pp->error_z = pp->target_z[pp->point_count] - pp->real_z;

		// I think code here is to calculate time to enable pp_lck_enb or not
		// If error < tol, add pp_lck
		if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z)
			pp->pp_lck = 1.0;
		else
			pp->pp_lck = 0.0;

		// Increase when no moving curve
		pp->pp_lck_count = pp->pp_lck_count + 1.0;

		if(pp->pp_lck_count <= 60.0)
			pp->pp_lck_cal = pp->pp_lck_cal + pp->pp_lck;
		else{
			pp->pp_lck_final = (pp->pp_lck_cal) / (pp->pp_lck_count) ;
			if(pp->pp_lck_final >= 0.95)
				pp->pp_lck_enb = 1;
			else
				pp->pp_lck_enb = 0;
			pp->pp_lck_count = 0.0;
			pp->pp_lck_cal   = 0.0;
		}


		if(((fabs(pp->error_x)+fabs(pp->error_y))/2) < pp->tol_xy && fabs(pp->error_z) < pp->tol_z){ // Already almost at destination
			if(pp->point_count < (pp->target_point - 2)){ // Checking is second last point or not
				if(pp->target_accurate[pp->point_count] == 1.0){ // point_lock enable
					if(pp->pp_lck_enb == 1){ // After < tol some time, lck_enb = 1
						pp->point_count++; // Showing reached previous destination
						pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
						pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
						pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
						if(pp->target_x[pp->point_count] - *(pp->lsrx)){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->target_x[pp->point_count] - *(pp->lsrx)),
									pp->ku_x[pp->point_count],
									pp->kp[0],
									pp->ki[0],
									pp->kd[0],
									30.0,
									&(pp->x));
							//								PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
						} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_x[pp->point_count],
									pp->kp[0],
									pp->ki[0],
									pp->kd[0],
									30.0,
									&(pp->x));
							//								PIDGainSet(KE,1.0,&(pp->x));
						}
						if(pp->target_y[pp->point_count]-pp->real_y){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
									pp->ku_y[pp->point_count],
									pp->kp[0],
									pp->ki[0],
									pp->kd[0],
									30.0,
									&(pp->y));
							//								PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
						} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_y[pp->point_count],
									pp->kp[0],
									pp->ki[0],
									pp->kd[0],
									30.0,
									&(pp->y));
							//								PIDGainSet(KE,1.0,&(pp->y));
						}

						//							PIDGainSet(KP,pp->kp[0],&(pp->x));
						//							PIDGainSet(KP,pp->kp[0],&(pp->y));
						//							PIDGainSet(KI,pp->ki[0],&(pp->x));
						//							PIDGainSet(KI,pp->ki[0],&(pp->y));
						//							PIDGainSet(KD,pp->kd[0],&(pp->x));
						//							PIDGainSet(KD,pp->kd[0],&(pp->y));
						//							PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
						//							PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
						pp->pp_lck_enb = 0;
					}
				}
				else{ // Already between tol, but point lck not enable
					pp->point_count++; // Assuming reached previous point
					pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
					pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
					pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
					if(pp->target_x[pp->point_count] - *(pp->lsrx)){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),
//									pp->ku_x[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->x));
						PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count] - *(pp->lsrx)),&(pp->x));
					} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_x[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->x));
						PIDGainSet(KE,1.0,&(pp->x));
					}
					if(pp->target_y[pp->point_count]-pp->real_y){
//							PIDGainInit(0.005,
//									1.0,
//									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
//									pp->ku_y[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->y));
						PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
					} else {
//							PIDGainInit(0.005,
//									1.0,
//									1.0,
//									pp->ku_y[pp->point_count],
//									pp->kp[0],
//									pp->ki[0],
//									pp->kd[0],
//									30.0,
//									&(pp->y));
						PIDGainSet(KE,1.0,&(pp->y));
					}
				}
			} else if(pp->point_count == (pp->target_point - 2)){ // Reached second last going last point
				if(pp->target_accurate[pp->point_count] == 1.0){ // point_lock enable
					if(pp->pp_lck_enb == 1){
						pp->point_count++;
						pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
						pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
						pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
						if(pp->target_x[pp->point_count] - *(pp->lsrx)){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->target_x[pp->point_count] - *(pp->lsrx)),
									pp->ku_x[pp->point_count],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->x));
							//								PIDGainSet(KE,1.0/fabs(pp->target_x[pp->point_count]-pp->real_x),&(pp->x));
						} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_x[pp->point_count],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->x));
							//								PIDGainSet(KE,1.0,&(pp->x));
						}
						if(pp->target_y[pp->point_count]-pp->real_y){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
									pp->ku_y[pp->point_count],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->y));
							//								PIDGainSet(KE,1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),&(pp->y));
						} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_y[pp->point_count],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->y));
							//								PIDGainSet(KE,1.0,&(pp->y));
						}
						pp->pp_lck_enb = 0;
					}
				}
				else{
					pp->point_count++;
					if(!pp->right_lsr)
						pp->error_x = pp->target_x[pp->point_count] - *(pp->lsrx);
					else
						pp->error_x = -(pp->target_x[pp->point_count] - *(pp->lsrR));
					pp->error_y = pp->target_y[pp->point_count] - pp->real_y;
					pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
					if(pp->error_x){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->error_x),
									pp->ku_x[0],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->x));
					} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_x[0],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->x));
					}
					if(pp->target_y[pp->point_count]-pp->real_y){
							PIDGainInit(0.005,
									1.0,
									1.0/fabs(pp->target_y[pp->point_count]-pp->real_y),
									pp->ku_y[pp->point_count],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->y));
					} else {
							PIDGainInit(0.005,
									1.0,
									1.0,
									pp->ku_y[0],
									pp->kp[1],
									pp->ki[1],
									pp->kd[1],
									30.0,
									&(pp->y));
					}
				}
			}else if(fabs(pp->error_x)<pp->f_tol_xy[0] && fabs(pp->error_y)<pp->f_tol_xy[0] && (int)pp->error_z<=pp->f_tol_z[0]){ // Reached last point
				led3 = 0;
				PIDDelayInit(&(pp->x));
				PIDDelayInit(&(pp->y));
				PIDDelayInit(&(pp->z));
				pp->pp_start=0;
				pp->lsr_start = 0;
				pp->pp_lck_enb = 0;
//				if(pp->target_accurate[pp->point_count] == 1.0){
//					if(pp->pp_lck_enb == 1){
////							RNSStop(&rns);
////							PP_stop(&pp);
//						led3 = 0;
//						//								APPStop();
//						//								LED4 = 1;
//						PIDDelayInit(&(pp->x));
//						PIDDelayInit(&(pp->y));
//						PIDDelayInit(&(pp->z));
//						pp->pp_start=0;
//						pp->lsr_start = 0;
//						pp->pp_lck_enb = 0;
//					}
//				}
//				else{
//					//							APPStop();
//					//							LED4 = 1;
//					led3 = 0;
////						RNSStop(&rns);
////						PP_stop(&pp);
//					PIDDelayInit(&(pp->x));
//					PIDDelayInit(&(pp->y));
//					PIDDelayInit(&(pp->z));
//					pp->pp_start=0;
//					pp->lsr_start = 0;
//				}
			}
		}

		if(pp->lsr_start){

			pp->dx = pp->error_x;
			pp->dy = pp->target_y[pp->point_count] - pp->prev_real_y;

			pp->rotate=0;

			if ((pp->dx != 0.0 || pp->dx != -0.0)&&(pp->dy != -0.0 || pp->dy != 0.0)){
				pp->heading = atan2f(pp->dy, pp->dx);
			} else {
				if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy < 0.0) { // Backward
					pp->heading = -1.5708;
				} else if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy > 0.0) { // Forward
					pp->heading = 1.5708;
				} else if((pp->dy == 0.0 || pp->dy == -0.0) && pp->dx > 0){ // Right
					pp->heading = 0.0;
//						pp->rotate = 1;
				}
				else if((pp->dy == 0.0 || pp->dy == -0.0) && pp->dx < 0) // Left
				{
					pp->heading = M_PI;
				}
			}

			pp->vx  = pp->target_vel[pp->point_count] * cosf(pp->heading);
			pp->vy  = pp->target_vel[pp->point_count] * sinf(pp->heading);

			if(pp->rotate){
				pp->vx = pp->vy = 0.0;
			}

			PID(&(pp->x));
			PID(&(pp->y));
			PID(&(pp->z));

			pp->rux =   pp->outx*cosf(pp->real_z_rad) - pp->outy*sinf(pp->real_z_rad);
			pp->ruy =   pp->outx*sinf(pp->real_z_rad) + pp->outy*cosf(pp->real_z_rad);
			pp->rvx =   pp->vx*cosf(pp->real_z_rad) - pp->vy*sinf(pp->real_z_rad);
			pp->rvy =   pp->vx*sinf(pp->real_z_rad) + pp->vy*cosf(pp->real_z_rad);

			if(pp->base_shape== fwd_omni){
					pp->u1 = 0.707107 * ( pp->ruy + pp->rux) + (pp->outz * 1.0);
					pp->u2 = 0.707107 * ( pp->ruy - pp->rux) - (pp->outz * 1.0);
					pp->u3 = 0.707107 * ( pp->ruy - pp->rux) + (pp->outz * 1.0);
					pp->u4 = 0.707107 * ( pp->ruy + pp->rux) - (pp->outz * 1.0);

//				pp->u1 = (pp->outz * 1.0);
//				pp->u2 = -pp->outz * 1.0;
//				pp->u3 = (pp->outz * 1.0);
//				pp->u4 = -(pp->outz * 1.0);


				pp->v1 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u1;
				pp->v2 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u2;
				pp->v3 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u3;
				pp->v4 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u4;

				//				pp->move(pp->v2,pp->v1,pp->v3,pp->v4,pp->rns);
			} else if(pp->base_shape== 1){

				pp->u1 = (0.866 * pp->ruy) - (0.5 * pp->rux) + (pp->outz * 1.0);
				pp->u2 = (0.866 * pp->ruy) + (0.5 * pp->rux) - (pp->outz * 1.0);
				pp->u3 = 1.0 * pp->rux + (pp->outz * 1.0);
				pp->v1 = (0.866 * pp->rvy) - (0.5 * pp->rvx) + pp->u1;
				pp->v2 = (0.866 * pp->rvy) + (0.5 * pp->rvx) + pp->u2;
				pp->v3 = 1.0 * pp->rvx + pp->u3;
				//				pp->move(pp->v2,pp->v1,pp->v3,0.0,pp->rns);
			}
//				hb_count = HAL_GetTick();

		}
	}
//	else if(pp->point_start && pp->lsr_start){
//		// lsrx is LeftX while lsry is RightX
//		// wont need to calculate error_y in this code
//			pp->real_lsrx= *(pp->lsrx) * cos(((pp->target_z[pp->point_count]-pp->real_z)*3.142)/180.0); // real_lsrx should always same with lsrx without rotating
//			pp->real_lsry= *(pp->lsry) * cos(((pp->target_z[pp->point_count]-pp->real_z)*3.142)/180.0);
//			if(!pp->lsr_init){
//				pp->error_x=0;
//				pp->error_y=0;
//			}else{
//				pp->error_x = pp->real_lsrx - pp->target_x[pp->point_count];
//				pp->error_y = pp->real_lsry - pp->target_y[pp->point_count];
//				if(pp->lsrx_swap)
//					pp->error_x=-pp->error_x;
//				if(pp->lsry_swap)
//					pp->error_y=-pp->error_y;
//			}
//			pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
//
//			if(fabs(pp->error_x) < pp->f_tol_xy[pp->point_count] && fabs(pp->error_y) < pp->f_tol_xy[pp->point_count] && fabs(pp->error_z) < pp->f_tol_z[pp->point_count])
//				pp->pp_lck = 1.0;
//			else
//				pp->pp_lck = 0.0;
//
//			pp->pp_lck_count = pp->pp_lck_count + 1.0;
//
//			if(pp->pp_lck_count <= 60.0)
//				pp->pp_lck_cal = pp->pp_lck_cal + pp->pp_lck;
//			else{
//				pp->pp_lck_final = (pp->pp_lck_cal) / (pp->pp_lck_count) ;
//				if(pp->pp_lck_final >= 0.95)
//					pp->pp_lck_enb = 1;
//				else
//					pp->pp_lck_enb = 0;
//				pp->pp_lck_count = 0.0;
//				pp->pp_lck_cal   = 0.0;
//			}
//
//			if(!pp->lsr_init){
//				if(fabs(pp->error_z) < pp->f_tol_z[0]){
//					pp->lsr_init=1;
//					pp->error_x = pp->real_lsrx - pp->target_x[0];
//					pp->error_y = pp->real_lsry - pp->target_y[0];
//					if(pp->lsrx_swap)
//						pp->error_x=-pp->error_x;
//					if(pp->lsry_swap)
//						pp->error_y=-pp->error_y;
//					if(pp->single){
//						if (pp->inertia >= sqrt(pp->error_x * pp->error_x + pp->error_y * pp->error_y)){
//							pp->single=0;
//							pp->point_count++;
//						}
//						pp->prev_error_x=pp->error_x;
//						pp->prev_error_y=pp->error_y;
//					}
//
//					pp->target_angle1[pp->point_count] = atanf(fabs( pp->error_y) / fabs(pp->error_x));
//
//					pp->ku_x[pp->point_count] = cosf(pp->target_angle1[pp->point_count]) * pp->ku_y[pp->point_count];
//					pp->ku_y[pp->point_count] *= sinf(pp->target_angle1[pp->point_count]);
//					//target angle calculated again to get speed of pid output
//					if(pp->single)
//						pp->target_angle1[0] = pp->ku_x[pp->point_count] / cosf(pp->target_angle1[pp->point_count]);
//					PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
//					PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
//					if(fabs(pp->error_x)){
//						PIDGainSet(KE,1.0/fabs(pp->error_x),&(pp->x));
//					} else {
//						PIDGainSet(KE,1.0,&(pp->x));
//					}
//					if(fabs(pp->error_y)){
//						PIDGainSet(KE,1.0/fabs(pp->error_y),&(pp->y));
//					} else {
//						PIDGainSet(KE,1.0,&(pp->y));
//					}
//
//					if(pp->point_count+1 == pp->target_point){
//
//						PIDGainSet(KP,pp->kp[1],&(pp->x));
//						PIDGainSet(KP,pp->kp[1],&(pp->y));
//						PIDGainSet(KI,pp->ki[1],&(pp->x));
//						PIDGainSet(KI,pp->ki[1],&(pp->y));
//						PIDGainSet(KD,pp->kd[1],&(pp->x));
//						PIDGainSet(KD,pp->kd[1],&(pp->y));
//					}else{
//
//						PIDGainSet(KP,pp->kp[0],&(pp->x));
//						PIDGainSet(KP,pp->kp[0],&(pp->y));
//						PIDGainSet(KI,pp->ki[0],&(pp->x));
//						PIDGainSet(KI,pp->ki[0],&(pp->y));
//						PIDGainSet(KD,pp->kd[0],&(pp->x));
//						PIDGainSet(KD,pp->kd[0],&(pp->y));
//					}
//
//					PIDDelayInit(&(pp->x));
//					PIDDelayInit(&(pp->y));
//					PIDDelayInit(&(pp->z));
//
//				}
//			}else{
//				if(pp->single){
//					pp->vel_ratio=sqrt((pp->prev_error_x - pp->error_x)*(pp->prev_error_x - pp->error_x) + (pp->prev_error_y - pp->error_y)*(pp->prev_error_y - pp->error_y))/(0.005 * pp->target_angle1[0]);
//					if (fabs(pp->vel_ratio)>pp->target_angle1[0] * 1.5)
//						pp->vel_ratio=pp->prev_vel_ratio;
//					else
//						pp->prev_vel_ratio=pp->vel_ratio;
//					pp->prev_error_x=pp->error_x;
//					pp->prev_error_y=pp->error_y;
//				}
//
//
//				if(pp->single && sqrt(pp->error_x*pp->error_x + pp->error_y*pp->error_y)< pp->inertia*pp->vel_ratio){
//					pp->point_count++;
//					pp->target_angle1[1]= atanf(fabs(pp->error_y) / fabs(pp->error_x));
//					pp->ku_x[1] = pp->ku_y[1]* cosf(pp->target_angle1[1]);
//					pp->ku_y[1] *= sinf(pp->target_angle1[1]);
//					PIDGainSet(KP,pp->kp[1],&(pp->x));
//					PIDGainSet(KP,pp->kp[1],&(pp->y));
//					PIDGainSet(KI,pp->ki[1],&(pp->x));
//					PIDGainSet(KI,pp->ki[1],&(pp->y));
//					PIDGainSet(KD,pp->kd[1],&(pp->x));
//					PIDGainSet(KD,pp->kd[1],&(pp->y));
//					PIDDelayInit(&(pp->x));
//					PIDDelayInit(&(pp->y));
//					PIDDelayInit(&(pp->z));
//					PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
//					PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
//					if(pp->error_x){
//						PIDGainSet(KE,1.0/fabs(pp->error_x),&(pp->x));
//					} else {
//						PIDGainSet(KE,1.0,&(pp->x));
//					}
//					if(pp->error_y){
//						PIDGainSet(KE,1.0/fabs(pp->error_y),&(pp->y));
//					} else {
//						PIDGainSet(KE,1.0,&(pp->y));
//					}
//					pp->single=0;
//				}else if(fabs(pp->error_x) < pp->f_tol_xy[pp->point_count] && fabs(pp->error_y) < pp->f_tol_xy[pp->point_count] && fabs(pp->error_z) < pp->f_tol_z[pp->point_count]){
//
//					if(pp->point_count + 1 < (pp->target_point)){
//						pp->point_count++;
//						pp->error_x = pp->real_lsrx - pp->target_x[pp->point_count];
//						pp->error_y = pp->real_lsry - pp->target_y[pp->point_count];
//						if(pp->lsrx_swap)
//							pp->error_x=-pp->error_x;
//						if(pp->lsry_swap)
//							pp->error_y=-pp->error_y;
//						pp->error_z = pp->target_z[pp->point_count] - pp->real_z;
//						pp->target_angle1[pp->point_count]= atanf(fabs(pp->error_y) / fabs(pp->error_x));
//						pp->ku_x[pp->point_count] = pp->ku_y[pp->point_count]* cosf(pp->target_angle1[pp->point_count]);
//						pp->ku_y[pp->point_count] *= sinf(pp->target_angle1[pp->point_count]);
//						if(pp->error_x){
//							PIDGainSet(KE,1.0/fabs(pp->error_x),&(pp->x));
//						} else {
//							PIDGainSet(KE,1.0,&(pp->x));
//						}
//						if(pp->error_y){
//							PIDGainSet(KE,1.0/fabs(pp->error_y),&(pp->y));
//						} else {
//							PIDGainSet(KE,1.0,&(pp->y));
//						}
//
//						if(pp->point_count == (pp->target_point - 1)){
//							PIDGainSet(KP,pp->kp[1],&(pp->x));
//							PIDGainSet(KP,pp->kp[1],&(pp->y));
//							PIDGainSet(KI,pp->ki[1],&(pp->x));
//							PIDGainSet(KI,pp->ki[1],&(pp->y));
//							PIDGainSet(KD,pp->kd[1],&(pp->x));
//							PIDGainSet(KD,pp->kd[1],&(pp->y));
//							PIDDelayInit(&(pp->x));
//							PIDDelayInit(&(pp->y));
//							PIDDelayInit(&(pp->z));
//						}
//
//						PIDGainSet(KU,pp->ku_x[pp->point_count],&(pp->x));
//						PIDGainSet(KU,pp->ku_y[pp->point_count],&(pp->y));
//
//					}else if(pp->pp_lck_enb == 1){
//						//trying for point lock
//
//						PP_stop(pp);
//						RNSStop(&rns);
//
//
//					}
//				}
//
//			}
//
//			if(pp->lsr_start){
//
//				pp->dx = pp->error_x;
//				pp->dy = pp->error_y;
//
//				pp->rotate=0;
//
//				if ((pp->dx != 0.0 || pp->dx != -0.0)&&(pp->dy != -0.0 || pp->dy != 0.0)){
//					pp->heading = atan2f(pp->dy, pp->dx);
//				} else {
//					if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy < 0.0) {
//						pp->heading = 1.5708;
//					} else if((pp->dx == 0.0 || pp->dx == -0.0) && pp->dy > 0.0) {
//						pp->heading = -1.5708;
//					} else {
//						pp->heading = 0.0;
//						pp->rotate = 1;
//					}
//				}
//
//				pp->vx  = pp->target_vel[pp->point_count] * cosf(pp->heading);
//				pp->vy  = pp->target_vel[pp->point_count] * sinf(pp->heading);
//
//				if(pp->rotate){
//					pp->vx = pp->vy = 0.0;
//				}
//
//				PID(&(pp->x));
//				PID(&(pp->y));
//				PID(&(pp->z));
//
//				pp->rux =   pp->outx*cosf(pp->real_z_rad) - pp->outy*sinf(pp->real_z_rad);
//				pp->ruy =   pp->outx*sinf(pp->real_z_rad) + pp->outy*cosf(pp->real_z_rad);
//				pp->rvx =   pp->vx*cosf(pp->real_z_rad) - pp->vy*sinf(pp->real_z_rad);
//				pp->rvy =   pp->vx*sinf(pp->real_z_rad) + pp->vy*cosf(pp->real_z_rad);
//
//				if(pp->base_shape== fwd_omni){
//					if(!pp->lsr_init){
//						pp->v1 = pp->u1 = - pp->outz;
//						pp->v2 = pp->u2 = pp->outz;
//						pp->v3 = pp->u3 = pp->outz;
//						pp->v4 = pp->u4 = - pp->outz;
//					}else{
//
//						pp->u1 = 0.707107 * ( pp->ruy - pp->rux) - (pp->outz * 1.0);
//						pp->u2 = 0.707107 * ( pp->ruy + pp->rux) + (pp->outz * 1.0);
//						pp->u3 = 0.707107 * ( pp->ruy - pp->rux) + (pp->outz * 1.0);
//						pp->u4 = 0.707107 * ( pp->ruy + pp->rux) - (pp->outz * 1.0);
//
//						pp->v1 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u1;
//						pp->v2 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u2;
//						pp->v3 = 0.707107 * ( pp->rvy - pp->rvx) + pp->u3;
//						pp->v4 = 0.707107 * ( pp->rvy + pp->rvx) + pp->u4;
//					}
//
//				} else if(pp->base_shape==tri_omni){
//
//					if(!pp->lsr_init){
//						pp->v1 = pp->u1 = pp->outz;
//						pp->v2 = pp->u2 = -pp->outz;
//						pp->v3 = pp->u3 = pp->outz;
//					}else{
//
//						pp->u1 = (0.866 * pp->ruy) - (0.5 * pp->rux) + (pp->outz * 1.0);
//						pp->u2 = (0.866 * pp->ruy) + (0.5 * pp->rux) - (pp->outz * 1.0);
//						pp->u3 = 1.0 * pp->rux + (pp->outz * 1.0);
//						pp->v1 = (0.866 * pp->rvy) - (0.5 * pp->rvx) + pp->u1;
//						pp->v2 = (0.866 * pp->rvy) + (0.5 * pp->rvx) + pp->u2;
//						pp->v3 = 1.0 * pp->rvx + pp->u3;
//					}
//				}
//				//			char nb[50];
//				//			sprintf(nb, "%.2f    %.2f    %.2f\n", pp->error_x, pp->error_y, pp->error_z);
//				//			HAL_UART_Transmit(&huart5, nb, strlen(nb), 15);
//				//			sprintf(nb, "v1: %.2f    v2:%.2f    z: %.2f\nv3: %.2f    v4: %.2f\n\n", pp->v1, pp->v2, pp->outz, pp->v3, pp->v4);
//				//			sprintf(nb, "rux: %.2f    ruy:%.2f    outx: %.2f   outy:%.2f\n", pp->rux, pp->ruy, pp->outx, pp->outy);
//
//
//			}
//
//		}




}



void PP_setXY (int x,int y,PathPlan_t *pp){

	QEIWrite(QEI2,x);
	QEIWrite(QEI5,y);

	//pp->real_x=0.0;
	//pp->real_y=0.0;

	//	pp->prev_real_x=0.0;
	//	pp->prev_real_y=0.0;

}

