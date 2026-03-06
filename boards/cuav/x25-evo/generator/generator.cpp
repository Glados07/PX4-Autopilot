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
 * @file generator.cpp
 *
 * Board-specific module to read ADC3 channel 5 (PF3 / ADC_6V6) on CUAV X25-EVO
 * and publish the generator bus voltage via uORB generator_status for
 * MAVLink GENERATOR_STATUS forwarding to QGC.
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
#include <math.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/generator_status.h>

using namespace time_literals;

/* Sampling interval */
static constexpr uint32_t GENERATOR_SAMPLE_INTERVAL_US = 100000; /* 10 Hz */

class Generator : public ModuleBase<Generator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Generator();
	~Generator() override = default;

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);

private:
	void Run() override;

	uORB::Publication<generator_status_s> _gen_pub{ORB_ID(generator_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::A6V6_V_DIV>) _param_v_div   ///< Generator ADC hardware voltage divider ratio
	)
};

Generator::Generator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void Generator::Run()
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
		ScheduleDelayed(GENERATOR_SAMPLE_INTERVAL_US);
		return;
	}

	/* Convert raw ADC value to voltage, then apply the hardware divider ratio
	 * to recover the true voltage at the ADC_6V6 port (0 ~ 6.6 V range).
	 */
	const float divider = _param_v_div.get();
	const float voltage = (static_cast<float>(raw) / static_cast<float>(px4_arch_adc_dn_fullcount()))
			      * px4_arch_adc_reference_v() * divider;

	/* Publish as generator_status → MAVLink GENERATOR_STATUS → QGC */
	generator_status_s gen{};
	gen.timestamp              = hrt_absolute_time();
	gen.status                 = generator_status_s::STATUS_FLAG_GENERATING;
	gen.bus_voltage             = voltage;
	gen.battery_current         = NAN;
	gen.load_current            = NAN;
	gen.power_generated         = NAN;
	gen.bat_current_setpoint    = NAN;
	gen.runtime                 = UINT32_MAX;
	gen.time_until_maintenance  = INT32_MAX;
	gen.generator_speed         = UINT16_MAX;
	gen.rectifier_temperature   = INT16_MAX;
	gen.generator_temperature   = INT16_MAX;
	_gen_pub.publish(gen);

	ScheduleDelayed(GENERATOR_SAMPLE_INTERVAL_US);
}

int Generator::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int Generator::task_spawn(int argc, char *argv[])
{
	Generator *instance = new Generator();

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

int Generator::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Reads the ADC_6V6 analog input (PF3 / ADC3 channel 5) and publishes the
generator bus voltage via generator_status uORB.

The hardware voltage divider ratio is set via the A6V6_V_DIV parameter.

This is forwarded over MAVLink as GENERATOR_STATUS and can be viewed in
QGC → Analyze Tools → MAVLink Inspector.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("generator", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int generator_main(int argc, char *argv[])
{
	return Generator::main(argc, argv);
}
