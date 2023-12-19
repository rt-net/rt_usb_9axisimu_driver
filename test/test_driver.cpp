
#include <gtest/gtest.h>
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu_driver.hpp"
#include "rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp"

#include "fakeit.hpp"
using namespace fakeit;
using namespace rt_usb_9axisimu;

TEST(TestDriver, startCommunication)
{
  // Mock settings
  // TODO(ShotAk): Use test fixture
  Mock<SerialPortInterface> mock;
  When(Method(mock, setPort).Using(Any())).AlwaysReturn();
  When(Method(mock, openPort)).Return(true);
  When(Method(mock, openSerialPort)).Return(true);
  When(Method(mock, closeSerialPort));
  When(Method(mock, readFromDevice)).AlwaysReturn(0);
  When(Method(mock, writeToDevice)).AlwaysReturn(0);

  // Set mock method for this test
  When(Method(mock, openSerialPort)).Return(true, false);

  std::unique_ptr<SerialPortInterface> mock_ptr(&mock.get());
  RtUsb9axisimuRosDriver driver(std::move(mock_ptr), "/dev/ttyUSB0");

  EXPECT_EQ(driver.startCommunication(), true);
  EXPECT_EQ(driver.startCommunication(), false);
}
