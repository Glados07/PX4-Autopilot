/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file adc6v6.cpp
 *
 * Board-specific module to read ADC3 channel 5 (PF3 / ADC_6V6) on CUAV X25-EVO
 * and publish the voltage via uORB debug_key_value for MAVLink forwarding to QGC.
 *
 * The hardware voltage divider ratio is configurable via the A6V6_V_DIV parameter.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <drivers/drv_adc.h>
#include <px4_arch/adc.h>
#include <board_config.h>
#include <string.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>

using namespace time_literals;

/* Sampling interval */
static constexpr uint32_t ADC6V6_SAMPLE_INTERVAL_US = 100000; /* 10 Hz */

class ADC6V6 : public ModuleBase<ADC6V6>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ADC6V6();
	~ADC6V6() override = default;

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);

private:
	void Run() override;

	uORB::Publication<debug_key_value_s> _debug_pub{ORB_ID(debug_key_value)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::A6V6_V_DIV>) _param_v_div   ///< ADC hardware voltage divider ratio
	)
};

ADC6V6::ADC6V6() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void ADC6V6::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	/* Update parameters if changed */
	updateParams();

	/* Sample ADC3 channel 5 (PF3 / ADC_6V6) */
	uint32_t raw = px4_arch_adc_sample(STM32_ADC3_BASE, ADC_ADC3_6V6_CHANNEL);

	if (raw == UINT32_MAX) {
		/* ADC read failed, retry later */
		ScheduleDelayed(ADC6V6_SAMPLE_INTERVAL_US);
		return;
	}

	/* Convert raw ADC value to voltage, then apply the hardware divider ratio
	 * to recover the true voltage at the ADC_6V6 port (0 ~ 6.6 V range).
	 */
	// const float divider = _param_v_div.get();
	// float voltage = (static_cast<float>(raw) / static_cast<float>(px4_arch_adc_dn_fullcount()))
	// 		* px4_arch_adc_reference_v() * divider;

	/* Publish as debug_key_value → MAVLink NAMED_VALUE_FLOAT → QGC */
	// debug_key_value_s dbg{};
	// dbg.timestamp = hrt_absolute_time();
	// strncpy(dbg.key, "ADC6V6", sizeof(dbg.key) - 1);
	// dbg.key[sizeof(dbg.key) - 1] = '\0';
	// dbg.value = voltage;
	// _debug_pub.publish(dbg);

	ScheduleDelayed(ADC6V6_SAMPLE_INTERVAL_US);
}

int ADC6V6::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int ADC6V6::task_spawn(int argc, char *argv[])
{
	ADC6V6 *instance = new ADC6V6();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	/* Ensure ADC3 is initialized (safe to call multiple times — no-op if already init'd) */
	if (px4_arch_adc_init(STM32_ADC3_BASE) != OK) {
		PX4_ERR("ADC3 init failed");
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}

	instance->ScheduleNow();
	return PX4_OK;
}

int ADC6V6::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reads the ADC_6V6 analog input (PF3 / ADC3 channel 5) and publishes the voltage
as a debug_key_value uORB message (key: "ADC6V6").

The hardware voltage divider ratio is set via the A6V6_V_DIV parameter.

This is forwarded over MAVLink as NAMED_VALUE_FLOAT and can be viewed in
QGC → Analyze Tools → MAVLink Inspector.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("adc6v6", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int adc6v6_main(int argc, char *argv[])
{
	return ADC6V6::main(argc, argv);
}
