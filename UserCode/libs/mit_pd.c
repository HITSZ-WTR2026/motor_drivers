/**
 * @file    mit_pd.c
 * @author  syhanjin
 * @date    2026-01-02
 * @brief   Brief description of the file
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
 * Project repository: https://github.com/HITSZ-WTRobot/motor_drivers
 */
#include "mit_pd.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

void MIT_PD_Init(MIT_PD_t* pd, const MIT_PD_Config_t* config)
{
    if (pd == NULL || config == NULL)
        return;

    pd->Kp             = config->Kp;
    pd->Kd             = config->Kd;
    pd->abs_output_max = config->abs_output_max;

    pd->p_ref = 0;
    pd->p_fdb = 0;
    pd->v_ref = 0;
    pd->v_fdb = 0;

    pd->output = 0;
}

void MIT_PD_Calculate(MIT_PD_t* pd)
{
    pd->output = pd->Kp * (pd->p_ref - pd->p_fdb) + pd->Kd * (pd->v_ref - pd->v_fdb);

    if (pd->output > pd->abs_output_max)
        pd->output = pd->abs_output_max;
    if (pd->output < -pd->abs_output_max)
        pd->output = -pd->abs_output_max;
}

#ifdef __cplusplus
}
#endif