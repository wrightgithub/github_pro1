#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"
#include "UserSys.h"

struct _pid{
        float kp;
			  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
			  float ki_out;
	      float kd_out;
	      float pid_out;
	
        int32_t _last_input;        ///< last input for derivative
				float _last_derivative;     ///< last derivative for low-pass filter
				float _derivative;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	

struct _ctrl{
		      u8  ctrlRate;
	        u8  ctrl_core_rate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };

extern struct _ctrl ctrl;						
						

void CONTROL(float rol, float pit, float yaw);
void PID_INIT(void);
void ALGH_set(void);
void Deblocking(void);

#endif
