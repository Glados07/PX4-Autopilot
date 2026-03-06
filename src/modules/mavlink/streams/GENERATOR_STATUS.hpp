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

#ifndef GENERATOR_STATUS_HPP
#define GENERATOR_STATUS_HPP

#include <uORB/topics/generator_status.h>

class MavlinkStreamGeneratorStatus : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGeneratorStatus(mavlink); }

	static constexpr const char *get_name_static() { return "GENERATOR_STATUS"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GENERATOR_STATUS; }

	const char *get_name() const override { return MavlinkStreamGeneratorStatus::get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _generator_status_sub.advertised() ? MAVLINK_MSG_ID_GENERATOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamGeneratorStatus(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _generator_status_sub{ORB_ID(generator_status)};

	bool send() override
	{
		generator_status_s gen;

		if (_generator_status_sub.update(&gen)) {
			mavlink_generator_status_t msg{};

			msg.status                = gen.status;
			msg.battery_current       = gen.battery_current;
			msg.load_current          = gen.load_current;
			msg.power_generated       = gen.power_generated;
			msg.bus_voltage           = gen.bus_voltage;
			msg.bat_current_setpoint  = gen.bat_current_setpoint;
			msg.runtime               = gen.runtime;
			msg.time_until_maintenance = gen.time_until_maintenance;
			msg.generator_speed       = gen.generator_speed;
			msg.rectifier_temperature = gen.rectifier_temperature;
			msg.generator_temperature = gen.generator_temperature;

			mavlink_msg_generator_status_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // GENERATOR_STATUS_HPP
