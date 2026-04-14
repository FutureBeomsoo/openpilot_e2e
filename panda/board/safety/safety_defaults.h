const addr_checks default_rx_checks = {
  .check = NULL,
  .len = 0,
};

int default_rx_hook(CANPacket_t *to_push) {
  UNUSED(to_push);
  return true;
}

// *** no output safety mode ***

static const addr_checks* nooutput_init(uint16_t param) {
  UNUSED(param);
  return &default_rx_checks;
}

static int nooutput_tx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);
  return false;
}

static int nooutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return false;
}

static int default_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;

  // Pre-init SW bridge for 3-bus SPAS layout:
  //   Bus 0 (Chassis) <-> Bus 2 (Camera+SCC): physically bridged via NC relay → no SW needed
  //   Bus 1 (MDPS+Radar): physically isolated → must be bridged in SW before hyundai_fwd_hook takes over
  //
  // Fault chain without this fix:
  //   SCC checks LKAS → LKAS checks MDPS → MDPS signal absent → SCC fault → SCC dead

  // Bus 0 → Bus 1: Chassis messages to MDPS (replicates 순정 연결)
  if (bus_num == 0) {
    bus_fwd = 2;  // Bus 1 (bitmask bit1)
  }

  // Bus 1 → Bus 0 + Bus 2: MDPS signals for LKAS/SCC health checks
  // Only MDPS12(593), SAS11(688), MDPS11(897) — radar frames are NOT forwarded
  if (bus_num == 1) {
    if (addr == 593 || addr == 688 || addr == 897) {
      bus_fwd = 5;  // 0b101 = Bus 0 + Bus 2
    }
  }

  return bus_fwd;
}

const safety_hooks nooutput_hooks = {
  .init = nooutput_init,
  .rx = default_rx_hook,
  .tx = nooutput_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = default_fwd_hook,
};

// *** all output safety mode ***

// Enables passthrough mode where relay is open and bus 0 gets forwarded to bus 2 and vice versa
const uint16_t ALLOUTPUT_PARAM_PASSTHROUGH = 1;
bool alloutput_passthrough = false;

static const addr_checks* alloutput_init(uint16_t param) {
  controls_allowed = true;
  alloutput_passthrough = GET_FLAG(param, ALLOUTPUT_PARAM_PASSTHROUGH);
  return &default_rx_checks;
}

static int alloutput_tx_hook(CANPacket_t *to_send) {
  UNUSED(to_send);
  return true;
}

static int alloutput_tx_lin_hook(int lin_num, uint8_t *data, int len) {
  UNUSED(lin_num);
  UNUSED(data);
  UNUSED(len);
  return true;
}

static int alloutput_fwd_hook(int bus_num, int addr) {
  int bus_fwd = -1;
  UNUSED(addr);

  if (alloutput_passthrough) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      bus_fwd = 0;
    }
  }

  return bus_fwd;
}

const safety_hooks alloutput_hooks = {
  .init = alloutput_init,
  .rx = default_rx_hook,
  .tx = alloutput_tx_hook,
  .tx_lin = alloutput_tx_lin_hook,
  .fwd = alloutput_fwd_hook,
};
