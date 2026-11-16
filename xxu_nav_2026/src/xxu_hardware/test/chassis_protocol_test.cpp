#include "xxu_hardware/chassis_protocol.hpp"
#include <gtest/gtest.h>
#include <iomanip>
#include <sstream>
using namespace xxu_hardware;
namespace {
std::string hex(const std::vector<std::uint8_t> & bytes)
{std::ostringstream out; for (auto b : bytes) {out << std::hex << std::setw(2) << std::setfill('0') << int(b);} return out.str();}
ChassisStatus healthy(std::uint16_t ack = 0, std::uint32_t ms = 100)
{return {1, 35, ack, 1000, 0, 0, ms};}
}
TEST(ChassisProtocol, MatchesExistingPythonCodecGoldenFrames)
{
  EXPECT_EQ(hex(command_frame(1, false)), "aa5502100100070000000000000070da");
  EXPECT_EQ(hex(command_frame(65535, true, 1.2345, -.0015, .5)), "aa550210ffff0701d304fefff4012563");
  EXPECT_EQ(quantize(-.0005), -1);
  EXPECT_THROW(quantize(INFINITY), std::invalid_argument);
  EXPECT_THROW(quantize(32.768), std::invalid_argument);
}
TEST(ChassisProtocol, StopAckMustAdvancePastBaseline)
{
  ChassisStateMachine state;
  state.observe(healthy(), 0.);
  state.next(.01, 1., 0., 0., true); const auto stop = state.last_tx_seq;
  EXPECT_FALSE(state.running); EXPECT_FALSE(state.last_tx_run);
  state.observe(healthy(), .02); EXPECT_FALSE(state.running);
  state.observe(healthy(stop), .03); EXPECT_TRUE(state.running);
  state.next(.04, 1., 0., 0., true); EXPECT_TRUE(state.last_tx_run);
  state.next(.05, 1., 0., 0., false); EXPECT_FALSE(state.last_tx_run);
}
TEST(ChassisProtocol, StatusTimeoutAckStallFaultAndRebootFailClosed)
{
  auto unlock = [](ChassisStateMachine & s) {
      s.observe(healthy(), 0.); s.next(.01, 0, 0, 0, false); s.observe(healthy(s.last_tx_seq), .02);
    };
  ChassisStateMachine s; unlock(s); ASSERT_TRUE(s.running);
  s.next(.18, 1, 0, 0, true); EXPECT_FALSE(s.running); EXPECT_EQ(s.reason, "status_rx_timeout");
  ChassisStateMachine a; unlock(a);
  a.observe(healthy(a.last_tx_seq), .26); a.next(.28, 1, 0, 0, true);
  EXPECT_FALSE(a.running); EXPECT_EQ(a.reason, "cmd_ack_stall");
  ChassisStateMachine b; unlock(b); b.observe(healthy(0, 0), .03);
  EXPECT_FALSE(b.running); EXPECT_EQ(b.reason, "stm32_reboot_detected");
  ChassisStateMachine c; unlock(c); auto fault = healthy(c.last_tx_seq); fault.status |= 4;
  c.observe(fault, .03); EXPECT_FALSE(c.running);
  c.observe(healthy(c.last_tx_seq), .04); EXPECT_FALSE(c.running);
}
