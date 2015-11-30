#ifndef _STATUS_H_
#define _STATUS_H_
#include "mode.h"
#include "track_line.h"
void AltHold_Mode_Ctr(_MODE_STEP *alt_hold);
void Set_Init_Par(void);
void Fixed_Point_Mode_Ctr(_MODE_STEP *fixed_point_process);
void Track_Line_Process(_MODE_STEP *track_line_process);
void Reset_All_Pid_Par(void);
void Auto_Land(u8 circle_num );
void Track_line_ABC(_MODE_STEP *line_ABC_process);
void Track_line_ABA(_MODE_STEP *line_ABA_process);
#endif

