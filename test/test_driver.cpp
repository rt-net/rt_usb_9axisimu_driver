/*
 * test_driver.cpp
 *
 * License: BSD-3-Clause
 *
 * Copyright (c) 2015-2023 RT Corporation <support@rt-net.jp>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of RT Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <gtest/gtest.h>
#include "fakeit.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"

using fakeit::Mock;
using fakeit::When;
using rt_usb_9axisimu::SerialPort;

Mock<SerialPort> create_serial_port_mock(void) {
  Mock<SerialPort> mock;
  When(Method(mock, setPort)).AlwaysReturn();
  When(Method(mock, openPort)).AlwaysReturn(true);
  When(Method(mock, openSerialPort)).AlwaysReturn(true);
  When(Method(mock, closeSerialPort)).AlwaysReturn();
  When(Method(mock, readFromDevice)).AlwaysReturn(0);
  When(Method(mock, writeToDevice)).AlwaysReturn(0);
  return mock;
}

TEST(TestDriver, startCommunication)
{
  // Expect the startCommunication method to be called twice and return true then false
  auto mock = create_serial_port_mock();
  When(Method(mock, openSerialPort)).Return(true, false);  // Return true then false

  RtUsb9axisimuRosDriver driver(
    std::unique_ptr<SerialPort>(&mock.get()));

  EXPECT_TRUE(driver.startCommunication());
  EXPECT_FALSE(driver.startCommunication());
}
