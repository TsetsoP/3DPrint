/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../lcd/ultralcd.h"
#include "../../../libs/vector_3.h"

#if HAS_LEVELING
#include "../../feature/bedlevel/bedlevel.h"
#endif

#define ERROR_PARAMETERS "M1001 bad parameters."
#define ERROR_ROTATION "Object rotation too big."
#define ERROR_SIZE "Object size is invalid"
#define ERROR_OUT_OF_BED "Object is out of bed"

#define BUMP 5
#define SLOW_PROBE_FEEDRATE 10
#define MEASURE_POINTS 2
#define DIR_POSITIVE 1
#define DIR_NEGATIVE -1
#define NUMBER_SIZES 4
#define X_STEP_IDX 0
#define Y_STEP_IDX 1

enum MeasureErrors 
{
	MEASURE_NO_ERROR, OBJ_ROTATED_ERROR, OBJ_SIZE_INVALID_ERROR, OBJ_OUT_OF_BED
};

static bool parse_params(float &a, float &b);
static void reset_z();
static bool is_probe_hit();
static void homeaxises();
static void find_first_point(int y_step, xy_float_t &p1) ;
static void move_axis_to(const AxisEnum axis, int pos, feedRate_t feedrate);
static void probe_axis(const AxisEnum axis, int pos, feedRate_t feedrate);
static bool do_step(const AxisEnum axis, float steps[], int8_t dir);
//static MeasureErrors measure_dimentions(float a_side, float b_side);
static float axis_pos_at_point(const AxisEnum axis, xy_float_t &point);

bool parse_params(float &a, float &b)
{
	if (!parser.seen('A'))
	{
		return false;
	}
	a = parser.value_linear_units();

	if (!parser.seen('B'))
	{
		return false;
	}
	b = parser.value_linear_units();
 
	return true;
}

void reset_z()
{
	const float d = current_position[Z_AXIS] + position_shift[Z_AXIS];
	if (!NEAR_ZERO(d))
	{
		position_shift[Z_AXIS] -= d;
		update_workspace_offset(Z_AXIS);
	}
}

bool is_probe_hit()
{
	return (endstops.state() & (1 << X_Y_LASER_PROBE)) > 0;
}

void homeaxises()
{
	planner.synchronize();
	set_bed_leveling_enabled(false);
	remember_feedrate_scaling_off();
	endstops.enable(true);
	homeaxis(X_AXIS);
	homeaxis(Y_AXIS);
	reset_z();
	sync_plan_position();
	endstops.not_homing();
	restore_feedrate_and_scaling();
	ui.refresh();
}

void find_first_point(int y_step, xy_float_t &p1) 
{ 
	int max_pos_x =  max_length(X_AXIS);
	int max_pos_y =  max_length(Y_AXIS) - y_step;
    int x_pos = max_pos_x;

	while (current_position.y < max_pos_y)
	{
		probe_axis(X_AXIS, x_pos, feedRate_t(XY_PROBE_FEEDRATE_MM_S));
		if (is_probe_hit())
		{
			break;
		}
		probe_axis(Y_AXIS, current_position.y + y_step, feedRate_t(XY_PROBE_FEEDRATE_MM_S));
		x_pos = x_pos > 0 ? 0 : max_pos_x;
	}
	p1.x = current_position.x;
	p1.y = current_position.y;
}

void move_axis_to(const AxisEnum axis, int pos, feedRate_t feedrate)
{
	switch (axis)
	{
	case X_AXIS:
		do_blocking_move_to_x(pos, feedrate);
		break;
	case Y_AXIS:
		do_blocking_move_to_y(pos, feedrate);
		break;
	default:
		break;
	}
}

void probe_axis(const AxisEnum axis, int pos, feedRate_t feedrate)
{
	endstops.enable(true);
	move_axis_to(axis, pos, feedrate);
	endstops.hit_on_purpose(); 
	set_current_from_steppers_for_axis(axis);
	sync_plan_position();
	endstops.not_homing();
}

float axis_current_pos(const AxisEnum axis)
{
	switch (axis)
	{
	case X_AXIS:
		return current_position.x;
		break;
	case Y_AXIS:
		return current_position.y;
		break;
	default:
		break;
	}
	return 0;
}

float axis_pos_at_point(const AxisEnum axis, xy_float_t &point)
{
	switch (axis)
	{
	case X_AXIS:
		return point.x;
		break;
	case Y_AXIS:
		return point.y;
		break;
	default:
		break;
	}
	return 0;
}

int8_t measure_bump_dir(int8_t size_idx)
{
	return size_idx == 0 || size_idx == 3 ? DIR_NEGATIVE : DIR_POSITIVE;
}

float axis_max_mesure_pos(const AxisEnum axis, xy_float_t &point, int8_t side_idx)
{
	float target_pos = axis_pos_at_point(axis, point);
	float max_pos = max_length(axis);
	if (target_pos == 0)
	{
		target_pos = max_pos;
	}
	else
	{
		target_pos -= (BUMP * measure_bump_dir(side_idx));

		if (target_pos > max_pos)
		{
			target_pos = max_pos;
		}
		if (target_pos < 0)
		{
			target_pos = 0;
		}
	}
	return target_pos;
}

bool mesure_axis(const AxisEnum axis, float max_pos, int8_t size_idx, xy_float_t &messured)
{
	probe_axis(axis, max_pos, get_homing_bump_feedrate(axis));
	if (!is_probe_hit())
	{
		return false;
	}
	move_axis_to(axis, axis_current_pos(axis) + (BUMP*measure_bump_dir(size_idx)), get_homing_bump_feedrate(axis));
	probe_axis(axis, max_pos, SLOW_PROBE_FEEDRATE);

	if (is_probe_hit())
	{
		messured.x = current_position.x;
		messured.y = current_position.y;
		return true;
	}
	return false;
}

bool do_step(const AxisEnum axis, float steps[], int8_t side_idx)
{
	int8_t dir = side_idx == 0 || side_idx == 3 ? DIR_NEGATIVE : DIR_POSITIVE;
	move_axis_to(axis, axis_current_pos(axis) + (BUMP*dir),  get_homing_bump_feedrate(axis));
	dir = side_idx >= 2 ? DIR_NEGATIVE : DIR_POSITIVE;
	if (axis == X_AXIS)
	{
		probe_axis(Y_AXIS,  axis_current_pos(Y_AXIS) + (steps[Y_STEP_IDX]*dir), feedRate_t(XY_PROBE_FEEDRATE_MM_S));
	}
	else
	{
		probe_axis(X_AXIS,  axis_current_pos(X_AXIS) + (steps[X_STEP_IDX]*dir), feedRate_t(XY_PROBE_FEEDRATE_MM_S));
	}
	
	return is_probe_hit();
}
 
void move_to_side_beginning(const AxisEnum axis, xy_float_t last_point, float steps[], int8_t side_idx)
{
	if (side_idx >= NUMBER_SIZES)
	{
		return;
	}
	xy_float_t target_point;
	int8_t dir = side_idx >= 2? DIR_NEGATIVE : DIR_POSITIVE;
	if(axis == X_AXIS)
	{
		float beginnig_offset =  (steps[Y_STEP_IDX] / 2)*dir;
		target_point.x = current_position.x;
		target_point.y = last_point.y + beginnig_offset;
	}
	else
	{
		float beginnig_offset = (steps[X_STEP_IDX] / 2)*dir;
		target_point.x = last_point.x + beginnig_offset;
		target_point.y = current_position.y;
	}
	do_blocking_move_to(target_point, get_homing_bump_feedrate(axis));
}

MeasureErrors measure_dimentions(float a_side, float b_side, xy_float_t measured_points[][MEASURE_POINTS]) 
{
	float steps[] = {a_side / (MEASURE_POINTS + 1),  b_side / (MEASURE_POINTS + 1)};
	xy_float_t measured;
    find_first_point(steps[Y_STEP_IDX], measured);
	do_blocking_move_to_x(0);
    AxisEnum probe_axis = X_AXIS;
	int side_index = 0;
	int point_index = 0;
 
	while (side_index < NUMBER_SIZES)
	{		
		float target_pos = axis_max_mesure_pos(probe_axis, measured, side_index);

		if (!mesure_axis(probe_axis, target_pos, side_index, measured))
		{
			if (point_index < MEASURE_POINTS)
			{
				return OBJ_SIZE_INVALID_ERROR;
			}

			if (side_index < NUMBER_SIZES)
			{
				point_index = 0;
				side_index++;
				probe_axis = probe_axis == X_AXIS ? Y_AXIS : X_AXIS;
				move_to_side_beginning(probe_axis, measured_points[side_index][point_index-1], steps, side_index);
				continue;
			}
		}
		else
		{
			if (measured.x <= 0 || measured.y <= 0)
			{
				 return OBJ_OUT_OF_BED;
			}
			
			if (point_index < MEASURE_POINTS)
			{
				measured_points[side_index][point_index].x = measured.x;
				measured_points[side_index][point_index].y = measured.y;
				point_index++;
			}
			else
			{
				measured_points[side_index][point_index-1].x = measured.x;
				measured_points[side_index][point_index-1].y = measured.y;
			}	
		}

		if (do_step(probe_axis, steps, side_index))
		{
			do_blocking_move_to_x(0);
			homeaxises();
			return OBJ_ROTATED_ERROR;
		}
	}
	return MEASURE_NO_ERROR;
}

xy_float_t find_intersection(xy_float_t a1, xy_float_t a2, xy_float_t b1, xy_float_t b2)
{
	xy_float_t int_point;
	float m1 = (a1.y-a2.y)/(a1.x-a2.x);
    float C1 = -m1*a1.x + a1.y;
    float m2 = (b1.y-b2.y)/(b1.x-b2.x);
	float C2 = -m2*b1.x + b1.y;
	int_point.x = (C1 - C2)/(m2 - m1);
	int_point.y = C2 + m2*int_point.x;
    return int_point;
}

//M1001 A 160 B 100
void GcodeSuite::M1001()
{
	float a_side, b_side;

	if (!parse_params(a_side, b_side))
	{
		SERIAL_ERROR_MSG(ERROR_PARAMETERS);
		return;
	}

	if (a_side <= 0 || b_side <= 0)
	{
		SERIAL_ERROR_MSG(ERROR_PARAMETERS);
	}

	ui.set_status("Start measure..", true);
	planner.reset_rotatation();
	homeaxises();
    
	xy_float_t measured_points[NUMBER_SIZES][MEASURE_POINTS];
//	measured_points[0][0].x = 14.2125006;
//	measured_points[0][0].y = 33;
//	measured_points[0][MEASURE_POINTS - 1].x = 18.0562496;
//	measured_points[0][MEASURE_POINTS - 1].y = 99;
//	measured_points[3][0].x = 168;
//	measured_points[3][0].y = 15.0749998;
//	measured_points[3][MEASURE_POINTS - 1].x = 60;
//	measured_points[3][MEASURE_POINTS - 1].y = 20.7250004;
	
	 endstops.enable(true);
	 endstops.enableXYLaserProbre(true);
	 HAL_adc_start(LASER_ADC, true);
	 MeasureErrors error = measure_dimentions(a_side, b_side, measured_points);
	 endstops.not_homing();
	 endstops.enableXYLaserProbre(false);
	 HAL_adc_start(LASER_ADC, false);

	switch (error)
	{
	case OBJ_ROTATED_ERROR:
		SERIAL_ERROR_MSG("Error:", ERROR_ROTATION);
		break;
	case OBJ_SIZE_INVALID_ERROR:
		SERIAL_ERROR_MSG("Error:", ERROR_SIZE);
		break;
	case OBJ_OUT_OF_BED:
		SERIAL_ERROR_MSG("Error:", ERROR_OUT_OF_BED);
		break;
	default:
		break;
	}

	xy_float_t origin_point = find_intersection(
		measured_points[3][MEASURE_POINTS - 1],
		measured_points[3][0],
		measured_points[0][0],
		measured_points[0][MEASURE_POINTS - 1]);

	do_blocking_move_to(origin_point, get_homing_bump_feedrate(X_AXIS));

	current_position.x = 0;
	current_position.y = 0;
	sync_plan_position();

	vector_3 vxy(origin_point - measured_points[3][0]);
	vxy.z=0;
	vxy.normalize();
    vxy *=-1;

	vector_3 vx(vxy.x, vxy.y, 0);
	vector_3 vy(-vxy.y, vxy.x, 0);
	vector_3 vz;

    matrix_3x3 R = matrix_3x3::create_from_rows(vx, vy, vz);
	planner.set_space_rotation(R);

	 position_shift[X_AXIS] += LASER_OFFSET_X;
	 update_workspace_offset(X_AXIS );
	 position_shift[Y_AXIS] += LASER_OFFSET_Y;
	 update_workspace_offset(Y_AXIS);

 	ui.set_status("Measure done", false);
}
