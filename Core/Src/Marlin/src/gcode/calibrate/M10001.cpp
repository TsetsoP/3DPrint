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

#if HAS_LEVELING
#include "../../feature/bedlevel/bedlevel.h"
#endif

#define ERROR_MESSAGE "M1001 bad parameters."

bool parse_params(xy_float_t &p1, xy_float_t &p2)
{
	if (!parser.seen('X'))
	{
		return false;
	}
	p1.x = parser.value_linear_units();

	if (!parser.seen('Y'))
	{
		return false;
	}
	p1.y = parser.value_linear_units();

	if (!parser.seen('I'))
	{
		return false;
	}
	p2.x = parser.value_linear_units();

	if (!parser.seen('J'))
	{
		return false;
	}
	p2.y = parser.value_linear_units();

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
	report_current_position();
}
//M1001 X 10 Y 20 I 30 J 50
void GcodeSuite::M1001()
{
	xy_float_t p1;
	xy_float_t p2;

	if (!parse_params(p1, p2))
	{
		SERIAL_ERROR_MSG(ERROR_MESSAGE);
		return;
	}

	if (p1.magnitude() > p2.magnitude())
	{
		SERIAL_ERROR_MSG(ERROR_MESSAGE);
	}
	homeaxises();

	xy_pos_t pos;
	pos.x = 100;
	pos.y = 0;
	endstops.enable(true);
	endstops.enableXYLaserProbre(true);
	HAL_adc_start(LASER_ADC, true);
	do_blocking_move_to(pos, 50);
	endstops.not_homing();
	endstops.enableXYLaserProbre(false);
	HAL_adc_start(LASER_ADC, false);

}
