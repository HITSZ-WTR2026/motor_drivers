/**
 * @file    pid_pd.c
 * @author  syhanjin
 * @date    2025-11-08
 *
 * Detailed description (optional).
 *
 * --------------------------------------------------------------------------
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * Project repository: https://github.com/HITSZ-WTRobot/chassises_controller
 */
#include "pid_pd.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

void PD_Calculate(PD_t* pd)
{
    pd->cur_error = pd->ref - pd->fdb;
    pd->output    = pd->Kp * pd->cur_error + pd->Kd * (pd->cur_error - pd->prev_error);
    if (pd->output > pd->abs_output_max)
        pd->output = pd->abs_output_max;
    if (pd->output < -pd->abs_output_max)
        pd->output = -pd->abs_output_max;

    pd->prev_error = pd->cur_error;
}

void PD_Init(PD_t* pd, const PD_Config_t* config)
{
    /* reset pd */
    memset(pd, 0, sizeof(PD_t));

    /* set pd arguments */
    pd->Kp             = config->Kp;
    pd->Kd             = config->Kd;
    pd->abs_output_max = config->abs_output_max;
}

#ifdef __cplusplus
}
#endif