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
 * ADC6V6 hardware voltage divider ratio.
 *
 * The ADC_6V6 rail voltage is divided down by a resistor divider to fit
 * the 0–3.3 V ADC input range. This parameter specifies the divider
 * ratio used to recover the original rail voltage from the ADC reading.
 *
 * For a 2:1 divider the ratio is 2.0 (default).
 *
 * @min 0.1
 * @max 20.0
 * @decimal 3
 * @group Fuel Level
 */
PARAM_DEFINE_FLOAT(A6V6_V_DIV, 2.0f);

/**
 * Fuel-signal ↔ ADC port voltage scale factor (k in y = k·x).
 *
 * port_voltage = k × fuel_signal_voltage.
 * The fuel-level module recovers the fuel signal as:
 *   fuel_signal = port_voltage / k
 *
 * Example: if the fuel sensor outputs 0–12 V and the ADC port maps
 * that to 0–6.6 V, then k = 6.6 / 12 = 0.55.
 *
 * Set to 1.0 when the fuel signal is connected directly without
 * additional scaling (fuel signal range equals ADC port range).
 *
 * @min 0.001
 * @max 100.0
 * @decimal 4
 * @group Fuel Level
 */
PARAM_DEFINE_FLOAT(FUEL_V_CONV_K, 1.0f);

/**
 * Fuel-signal voltage at full tank (V).
 *
 * The fuel sensor electrical signal voltage when the fuel tank
 * is completely full. Used together with FUEL_SIG_EMPT to map
 * the signal to a fuel percentage.
 *
 * @unit V
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @group Fuel Level
 */
PARAM_DEFINE_FLOAT(FUEL_SIG_FULL, 6.6f);

/**
 * Fuel-signal voltage at empty tank (V).
 *
 * The fuel sensor electrical signal voltage when the fuel tank
 * is completely empty. This is the offset "b" in the linear
 * relationship between signal voltage and fuel quantity.
 *
 * @unit V
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @group Fuel Level
 */
PARAM_DEFINE_FLOAT(FUEL_SIG_EMPT, 0.0f);

/**
 * Maximum fuel capacity (ml).
 *
 * The total fuel capacity of the tank in millilitres.
 * Used to convert the fuel percentage into an absolute
 * remaining-fuel value published in fuel_tank_status.
 *
 * @min 0.0
 * @max 1000000.0
 * @decimal 0
 * @group Fuel Level
 */
PARAM_DEFINE_FLOAT(FUEL_MAX_CAP, 1000.0f);

/**
 * Fuel type (MAV_FUEL_TYPE enum).
 *
 * 0 = Unknown, 1 = Liquid (ml), 2 = Gas (kPa).
 *
 * @value 0 Unknown
 * @value 1 Liquid
 * @value 2 Gas
 * @min 0
 * @max 2
 * @group Fuel Level
 */
PARAM_DEFINE_INT32(FUEL_TYPE, 1);
