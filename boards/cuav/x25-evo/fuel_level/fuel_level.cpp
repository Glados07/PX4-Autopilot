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
 * @file fuel_level.cpp
 *
 * Fuel-level estimation module for CUAV X25-EVO.
 *
 * Reads the ADC_6V6 analog input (PF3 / ADC3 channel 5) directly, applies
 * the hardware voltage-divider ratio, then a two-stage linear conversion
 * (ADC port voltage → fuel electrical signal → fuel quantity), and publishes
 * the result as fuel_tank_status for MAVLink FUEL_STATUS forwarding to QGC.
 *
 * The raw ADC port voltage is also published as a debug_key_value message
 * (key "ADC6V6") for monitoring in QGC MAVLink Inspector.
 *
 * Conversion chain
 * ────────────────
 *   0. raw ADC      →  port_voltage :  port_voltage = (raw / full_count) × V_ref × A6V6_V_DIV
 *
 *   1. port_voltage  →  fuel_signal  :  fuel_signal = port_voltage / FUEL_V_CONV_K
 *      (FUEL_V_CONV_K is the k in y = k·x  where y is port voltage, x is fuel signal)
 *
 *   2. fuel_signal   →  fuel_qty     :  fuel_qty = (fuel_signal − FUEL_SIG_EMPT)
 *                                                  ÷ (FUEL_SIG_FULL − FUEL_SIG_EMPT)
 *                                                  × FUEL_MAX_CAP
 *
 * All coefficients are exposed as PX4 parameters for easy tuning.
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
#include <string.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/fuel_tank_status.h>

using namespace time_literals;

/* Sampling / polling interval — 2 Hz is sufficient for fuel quantity */
static constexpr uint32_t FUEL_LEVEL_INTERVAL_US = 500000; /* 2 Hz */

class FuelLevel : public ModuleBase<FuelLevel>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	FuelLevel();
	~FuelLevel() override = default;

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);

private:
	void Run() override;

	uORB::Publication<fuel_tank_status_s>  _fuel_pub{ORB_ID(fuel_tank_status)};
	uORB::Publication<debug_key_value_s>   _debug_pub{ORB_ID(debug_key_value)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::A6V6_V_DIV>)     _param_v_div,      ///< ADC hardware voltage divider ratio
		(ParamFloat<px4::params::FUEL_V_CONV_K>)  _param_v_conv_k,   ///< y=kx scale factor  (port_v / fuel_sig)
		(ParamFloat<px4::params::FUEL_SIG_FULL>)   _param_sig_full,   ///< fuel-signal voltage at full  (V)
		(ParamFloat<px4::params::FUEL_SIG_EMPT>)   _param_sig_empt,   ///< fuel-signal voltage at empty (V)
		(ParamFloat<px4::params::FUEL_MAX_CAP>)    _param_max_cap,    ///< maximum fuel capacity (ml)
		(ParamInt<px4::params::FUEL_TYPE>)          _param_fuel_type   ///< MAV_FUEL_TYPE enum
	)
};

FuelLevel::FuelLevel() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void FuelLevel::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	/* Refresh tuneable parameters */
	updateParams();

	/* ── Read ADC3 channel 5 (PF3 / ADC_6V6) ── */
	uint32_t raw = px4_arch_adc_sample(STM32_ADC3_BASE, ADC_ADC3_6V6_CHANNEL);

	if (raw == UINT32_MAX) {
		/* ADC read failed, retry later */
		ScheduleDelayed(FUEL_LEVEL_INTERVAL_US);
		return;
	}

	/* Convert raw → port voltage (0–6.6 V) via hardware divider */
	const float divider = _param_v_div.get();
	const float port_voltage = (static_cast<float>(raw) / static_cast<float>(px4_arch_adc_dn_fullcount()))
				   * px4_arch_adc_reference_v() * divider;

	/* ── Stage 1: port voltage → fuel electrical signal ── */
	const float k = _param_v_conv_k.get();

	if (k < 1e-6f) {
		/* Avoid division-by-zero */
		ScheduleDelayed(FUEL_LEVEL_INTERVAL_US);
		return;
	}

	const float fuel_signal = port_voltage / k;

	/* ── Publish converted fuel signal voltage as debug_key_value for QGC monitoring ── */
	debug_key_value_s dbg{};
	dbg.timestamp = hrt_absolute_time();
	strncpy(dbg.key, "FUEL_ANALOG_Voltage", sizeof(dbg.key) - 1);
	dbg.key[sizeof(dbg.key) - 1] = '\0';
	dbg.value = fuel_signal;
	_debug_pub.publish(dbg);

	/* ── Stage 2: fuel signal → fuel quantity ── */
	const float sig_full  = _param_sig_full.get();
	const float sig_empty = _param_sig_empt.get();
	const float max_cap   = _param_max_cap.get();
	const float span      = sig_full - sig_empty;

	float percent = 0.0f;
	float remaining_ml = 0.0f;

	if (fabsf(span) > 1e-6f) {
		percent = (fuel_signal - sig_empty) / span;

		/* Clamp to [0, 1] */
		if (percent < 0.0f) { percent = 0.0f; }

		if (percent > 1.0f) { percent = 1.0f; }

		remaining_ml = percent * max_cap;
	}

	/* ── Publish fuel_tank_status ── */
	fuel_tank_status_s fuel{};
	fuel.timestamp              = hrt_absolute_time();
	fuel.maximum_fuel_capacity  = max_cap;
	fuel.consumed_fuel          = max_cap - remaining_ml;
	fuel.fuel_consumption_rate  = NAN;          /* not measured */
	fuel.percent_remaining      = static_cast<uint8_t>(percent * 100.0f + 0.5f);
	fuel.remaining_fuel         = remaining_ml;
	fuel.fuel_tank_id           = 0;
	fuel.fuel_type              = static_cast<uint32_t>(_param_fuel_type.get());
	fuel.temperature            = NAN;          /* not measured */

	_fuel_pub.publish(fuel);

	ScheduleDelayed(FUEL_LEVEL_INTERVAL_US);
}

int FuelLevel::custom_command(int argc, char *argv[])
{
	return print_usage("Unrecognized command.");
}

int FuelLevel::task_spawn(int argc, char *argv[])
{
	FuelLevel *instance = new FuelLevel();

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

int FuelLevel::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Fuel-level estimation module.

Reads the ADC_6V6 analog input (PF3 / ADC3 channel 5) directly, converts
the voltage to a fuel quantity through two configurable linear stages,
and publishes fuel_tank_status for MAVLink FUEL_STATUS forwarding.

The raw port voltage is also published as debug_key_value (key "ADC6V6")
for monitoring in QGC MAVLink Inspector.

#### Conversion stages

0. **Raw ADC → port voltage**
   port_voltage = (raw / full_count) × V_ref × A6V6_V_DIV

1. **Port voltage → fuel electrical signal**
   fuel_signal = port_voltage / FUEL_V_CONV_K

2. **Fuel signal → fuel quantity (ml)**
   quantity = (fuel_signal − FUEL_SIG_EMPT) / (FUEL_SIG_FULL − FUEL_SIG_EMPT) × FUEL_MAX_CAP

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fuel_level", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fuel_level_main(int argc, char *argv[])
{
	return FuelLevel::main(argc, argv);
}
