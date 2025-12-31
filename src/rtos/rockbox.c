// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Rockbox RTOS support
 *   Copyright (C) 2025 by Aidan MacDonald <amachronic@protonmail.com>
 *
 * Based on zephyr.c:
 *   Copyright (C) 2017 by Intel Corporation
 *   Leandro Pereira <leandro.pereira@intel.com>
 *   Daniel Glöckner <dg@emlix.com>*
 *   Copyright (C) 2021 by Synopsys, Inc.
 *   Evgeniy Didin <didin@synopsys.com>
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>

#include "helper/log.h"
#include "helper/types.h"
#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/target.h"
#include "target/armv7m.h"

enum rockbox_bininfo {
	BININFO_NUM_CORES,
	BININFO_NUM_THREADS,
	BININFO_OFFSET_CORE_RUNNING_THREAD,
	BININFO_OFFSET_THREADALLOC_AVAIL,
	BININFO_OFFSET_THREAD_CONTEXT,
	BININFO_OFFSET_THREAD_NAME_PTR,
	BININFO_OFFSET_THREAD_ID,
	BININFO_COUNT
};

struct rockbox_params {
	const char *target_name;
	uint8_t pointer_width;
	uint32_t num_cores;
	uint32_t num_threads;
	uint32_t switch_thread_pc;
	uint32_t bininfo[BININFO_COUNT];
	uint32_t bininfo_count;
	const struct rtos_register_stacking *stacking_info;
};

static const struct stack_register_offset rb_armv7m_registers[] = {
	{ ARMV7M_R0,   -1,   32 },
	{ ARMV7M_R1,   -1,   32 },
	{ ARMV7M_R2,   -1,   32 },
	{ ARMV7M_R3,   -1,   32 },
	{ ARMV7M_R4,   0x00, 32 },
	{ ARMV7M_R5,   0x04, 32 },
	{ ARMV7M_R6,   0x08, 32 },
	{ ARMV7M_R7,   0x0c, 32 },
	{ ARMV7M_R8,   0x10, 32 },
	{ ARMV7M_R9,   0x14, 32 },
	{ ARMV7M_R10,  0x18, 32 },
	{ ARMV7M_R11,  0x1c, 32 },
	{ ARMV7M_R12,  -1,   32 },
	{ ARMV7M_R13,  0x24, 32 }, /* sp */
	{ ARMV7M_R14,  0x20, 32 }, /* lr */
	{ ARMV7M_PC,   -1,   32 },
	{ ARMV7M_XPSR, -1,   32 },
};

static const struct rtos_register_stacking rb_armv7m_stacking = {
	.stack_registers_size = 40,
	.stack_growth_direction = -1,
	.num_output_registers = ARRAY_SIZE(rb_armv7m_registers),
	.register_offsets = rb_armv7m_registers,
};

static struct rockbox_params rockbox_params_list[] = {
	{
		.target_name = "cortex_m",
		.pointer_width = 4,
		.stacking_info = &rb_armv7m_stacking,
	},
	{
		.target_name = NULL
	}
};

enum rockbox_symbol_values {
	ROCKBOX_VAL_CORES,
	ROCKBOX_VAL_THREADS,
	ROCKBOX_VAL_THREADALLOC,
	ROCKBOX_VAL_SWITCH_THREAD_PC,
	ROCKBOX_VAL_BININFO,
	ROCKBOX_VAL_BININFO_COUNT,
	ROCKBOX_VAL_COUNT
};

static const struct symbol_table_elem rockbox_symbol_list[] = {
	[ROCKBOX_VAL_CORES]            = { .symbol_name = "__cores"                  },
	[ROCKBOX_VAL_THREADS]          = { .symbol_name = "__threads"                },
	[ROCKBOX_VAL_THREADALLOC]      = { .symbol_name = "threadalloc"              },
	[ROCKBOX_VAL_SWITCH_THREAD_PC] = { .symbol_name = "__rbocd_switch_thread_pc" },
	[ROCKBOX_VAL_BININFO]          = { .symbol_name = "__rbocd_bininfo"          },
	[ROCKBOX_VAL_BININFO_COUNT]    = { .symbol_name = "__rbocd_bininfo_count"    },
	[ROCKBOX_VAL_COUNT]            = { .symbol_name = NULL },
};

static bool rockbox_detect_rtos(struct target *target)
{
	if (!target->rtos->symbols)
		return false;

	for (int symbol = 0; symbol != ROCKBOX_VAL_COUNT; symbol++) {
		if (target->rtos->symbols[symbol].address == 0)
			return false;
	}

	return true;
}

static int rockbox_create(struct target *target)
{
	const char *name;

	name = target_type_name(target);

	for (struct rockbox_params *p = rockbox_params_list; p->target_name; p++) {
		if (!strcmp(p->target_name, name)) {
			LOG_INFO("Rockbox: target %s known", name);
			target->rtos->rtos_specific_params = p;
			return ERROR_OK;
		}
	}

	LOG_ERROR("Could not find target %s in Rockbox compatibility list", name);
	return ERROR_FAIL;
}

static int rockbox_read_running_thread_ptr(struct rtos *rtos,
					   uint32_t core_id,
					   uint32_t *thread_ptr)
{
	const struct rockbox_params *params = rtos->rtos_specific_params;
	uint32_t cores_addr = rtos->symbols[ROCKBOX_VAL_CORES].address;
	uint32_t core_addr = cores_addr + (params->pointer_width * core_id);

	if (core_id >= params->num_cores)
		return ERROR_FAIL;

	return target_read_u32(rtos->target,
			       core_addr + params->bininfo[BININFO_OFFSET_CORE_RUNNING_THREAD],
			       thread_ptr);
}

static int rockbox_read_thread_ptr(struct rtos *rtos,
				   uint32_t thread_id,
				   uint32_t *thread_ptr)
{
	const struct rockbox_params *params = rtos->rtos_specific_params;
	uint32_t threads_addr = rtos->symbols[ROCKBOX_VAL_THREADS].address;

	if (thread_id >= params->num_threads)
		return ERROR_FAIL;

	return target_read_u32(rtos->target,
			       threads_addr + (params->pointer_width * thread_id),
			       thread_ptr);
}

static int rockbox_read_thread_allocated(struct rtos *rtos,
					 uint32_t thread_id,
					 bool *allocated)
{
	const struct rockbox_params *params = rtos->rtos_specific_params;
	uint32_t threadalloc_addr = rtos->symbols[ROCKBOX_VAL_THREADALLOC].address;
	uint32_t threadalloc_avail = threadalloc_addr + params->bininfo[BININFO_OFFSET_THREADALLOC_AVAIL];
	uint32_t threadalloc_word = threadalloc_avail + 4 * (thread_id / 32);
	uint32_t threadalloc_bit = 1u << (thread_id % 32);

	if (thread_id >= params->num_threads)
		return ERROR_FAIL;

	uint32_t avail_bits;
	int retval = target_read_u32(rtos->target, threadalloc_word, &avail_bits);
	if (retval != ERROR_OK)
		return retval;

	*allocated = ((avail_bits & threadalloc_bit) == 0);
	return ERROR_OK;
}

static int rockbox_read_thread_name(struct rtos *rtos,
				    uint32_t thread_ptr,
				    char *name, size_t name_len)
{
	const struct rockbox_params *params = rtos->rtos_specific_params;
	int retval;

	uint32_t name_ptr;
	retval = target_read_u32(rtos->target,
				 thread_ptr + params->bininfo[BININFO_OFFSET_THREAD_NAME_PTR],
				 &name_ptr);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_buffer(rtos->target, name_ptr,
				    name_len - 1, (uint8_t *)name);
	if (retval != ERROR_OK)
		return retval;

	name[name_len - 1] = '\0';
	return ERROR_OK;
}

static int rockbox_update_threads(struct rtos *rtos)
{
	struct rockbox_params *params;
	int retval;

	if (!rtos->rtos_specific_params)
		return ERROR_FAIL;

	params = (struct rockbox_params *)rtos->rtos_specific_params;

	if (!rtos->symbols) {
		LOG_ERROR("No symbols for Rockbox");
		return ERROR_FAIL;
	}

	retval = target_read_u32(rtos->target,
				 rtos->symbols[ROCKBOX_VAL_BININFO_COUNT].address,
				 &params->bininfo_count);
	if (retval != ERROR_OK)
		return ERROR_FAIL;
	if (params->bininfo_count < BININFO_COUNT) {
		LOG_ERROR("Wrong bininfo_count, got %" PRIu32 ", need %d",
			  params->bininfo_count, BININFO_COUNT);
		return ERROR_FAIL;
	}

	for (uint32_t i = 0; i < params->bininfo_count; ++i)
	{
		retval = target_read_u32(rtos->target,
					 rtos->symbols[ROCKBOX_VAL_BININFO].address + (4 * i),
					 &params->bininfo[i]);
		if (retval != ERROR_OK)
			return retval;
	}

	params->switch_thread_pc = rtos->symbols[ROCKBOX_VAL_SWITCH_THREAD_PC].address;
	params->num_cores = params->bininfo[BININFO_NUM_CORES];
	params->num_threads = params->bininfo[BININFO_NUM_THREADS];

	uint32_t running_thread_ptr;
	retval = rockbox_read_running_thread_ptr(rtos, 0, &running_thread_ptr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not obtain running thread pointer");
		return ERROR_FAIL;
	}

	struct thread_detail *details = calloc(params->num_threads, sizeof(*details));
	if (!details)
		return ERROR_FAIL;

	rtos->thread_count = 0;

	for (uint32_t thread = 0; thread < params->num_threads; ++thread)
	{
		struct thread_detail *detail;
		uint32_t thread_ptr;
		uint32_t thread_id;
		char thread_name[64];
		bool is_alloc = false;

		retval = rockbox_read_thread_allocated(rtos, thread, &is_alloc);
		if (retval != ERROR_OK)
			goto error;

		if (!is_alloc)
			continue;

		retval = rockbox_read_thread_ptr(rtos, thread, &thread_ptr);
		if (retval != ERROR_OK)
			goto error;

		retval = target_read_u32(rtos->target,
					 thread_ptr + params->bininfo[BININFO_OFFSET_THREAD_ID],
					 &thread_id);
		if (retval != ERROR_OK)
			goto error;

		detail = &details[rtos->thread_count];
		detail->threadid = thread_ptr;
		detail->extra_info_str = alloc_printf("ID: %" PRIx32, thread_id);
		detail->exists = true;

		/* Get name, this is optional as it's behind a pointer */
		retval = rockbox_read_thread_name(rtos, thread_ptr,
						  thread_name, sizeof(thread_name));
		if (retval == ERROR_OK) {
			if (thread_name[0] == '\0')
				detail->thread_name_str = strdup("<empty>");
			else
				detail->thread_name_str = strdup(thread_name);
		} else {
			detail->thread_name_str = strdup("<inaccessible>");
		}

		if (thread_ptr == running_thread_ptr)
			rtos->current_thread = thread_ptr;

		rtos->thread_count++;
	}

	rtos->thread_details = realloc(details,
				       rtos->thread_count * sizeof(*details));
	return ERROR_OK;

error:
	free(details);
	return retval;
}

static int rockbox_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	struct rockbox_params *params = rtos->rtos_specific_params;
	uint32_t thread_ptr = thread_id;
	int retval;

	retval = rtos_generic_stack_read(rtos->target,
					 params->stacking_info,
					 thread_ptr + params->bininfo[BININFO_OFFSET_THREAD_CONTEXT],
					 reg_list, num_regs);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Set PC to fixed value, due to cooperative scheduling
	 * all non-current threads will appear stopped somewhere
	 * within switch_thread().
	 */
	for (int i = 0; i < params->stacking_info->num_output_registers; ++i)
	{
		if ((*reg_list)[i].number == ARMV7M_PC)
		{
			buf_cpy(&params->switch_thread_pc,
				(*reg_list)[i].value, (*reg_list)[i].size);
			break;
		}
	}

	return ERROR_OK;
}

static int rockbox_get_symbol_list_to_lookup(struct symbol_table_elem **symbol_list)
{
	*symbol_list = malloc(sizeof(rockbox_symbol_list));
	if (!*symbol_list) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	memcpy(*symbol_list, rockbox_symbol_list, sizeof(rockbox_symbol_list));
	return ERROR_OK;
}

const struct rtos_type rockbox_rtos = {
	.name = "Rockbox",

	.detect_rtos = rockbox_detect_rtos,
	.create = rockbox_create,
	.update_threads = rockbox_update_threads,
	.get_thread_reg_list = rockbox_get_thread_reg_list,
	.get_symbol_list_to_lookup = rockbox_get_symbol_list_to_lookup,
};
