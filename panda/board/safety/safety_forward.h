// Stores the array index of a matched car fingerprint/forwarding profile
int enabled = -1;
int MDPS12_checksum = -1;
int MDPS12_cnt = 0;   
int last_StrColT = 0;

static void forward_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);
  
  if ((bus == 0) && (addr == 832)) {
    hyundai_camera_detected = 1;
  }
  
  // Find out which bus the camera is on
  if ((bus != 0) && (addr == 832)) {
    hyundai_camera_bus = bus;
  }
  
  // 832 is lkas cmd. If it is on camera bus, then giraffe switch 2 is high
  if ((addr == 832) && (bus == hyundai_camera_bus) && (hyundai_camera_detected != 1)) {
    hyundai_giraffe_switch_2 = 1;
  }
  if (addr == 593) {
    if (MDPS12_checksum == -1) {
      int New_Chksum2 = 0;
      uint8_t dat[8];
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(to_push, i);
        }
      int Chksum2 = dat[3];
      dat[3] = 0;
      for (int i=0; i<8; i++) {
        New_Chksum2 += dat[i];
        }
      New_Chksum2 %= 256;
      if (Chksum2 == New_Chksum2) {
      MDPS12_checksum = 1;
      }
      else {
      MDPS12_checksum = 0;
      }
    }
  } 
  if ((enabled != 1) && (hyundai_camera_detected != 1) && (hyundai_giraffe_switch_2 == 1)) {
    safety_cb_enable_all();
    // begin forwarding with that profile
    enabled = 1;
    }
  if ((enabled == 1) && (hyundai_camera_detected == 1)) {
    // camera connected, disable forwarding
    enabled = 0;
    safety_cb_disable_all();
    }

}

static int forward_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {
  int addr = GET_ADDR(to_send);
  if (enabled == 1) {
    if (addr == 593) {
      
      uint8_t dat[8];
      int New_Chksum2 = 0;
      for (int i=0; i<8; i++) {
        dat[i] = GET_BYTE(to_send, i);
      }
      if (MDPS12_cnt > 330) {
        int StrColTq = dat[0] | (dat[1] & 0x7) << 8;
        int OutTq = dat[6] >> 4 | dat[7] << 4;
        if (MDPS12_cnt == 331) {
          StrColTq -= 164;
        } else {
          StrColTq = last_StrColT + 34;
        }
        OutTq = 2058;

        dat[0] = StrColTq & 0xFF;
        dat[1] &= 0xF8;
        dat[1] |= StrColTq >> 8;
        dat[6] &= 0xF;
        dat[6] |= (OutTq & 0xF) << 4;
        dat[7] = OutTq >> 4;
            

        to_send->RDLR &= 0xFFF800;
        to_send->RDLR |= StrColTq;
        to_send->RDHR &= 0xFFFFF;
        to_send->RDHR |= OutTq << 20;
        last_StrColT = StrColTq;
        }
      dat[3] = 0;
      if (MDPS12_checksum) { 
        for (int i=0; i<8; i++) {
          New_Chksum2 += dat[i];
        }
        New_Chksum2 %= 256;

      } else if (!MDPS12_checksum) { //we need CRC8 checksum
        uint8_t crc = 0xFD;
        uint16_t poly = 0x11D;
        int i, j;
        for (i=0; i<8; i++){
          crc ^= dat[i];
          for (j=0; j<8; j++) {
            if ((crc & 0xDF) != 0U) {
              crc = (uint8_t)((crc << 1) ^ poly);
            } else {
              crc <<= 1;
            }
          }
        }
        New_Chksum2 = crc;
      }
      to_send->RDLR |= New_Chksum2 << 24;
      MDPS12_cnt += 1;
      MDPS12_cnt %= 345;
      }
      // must be true for fwd_hook to function
      return 1;
  }
  return 0;
}

static int forward_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  UNUSED(to_fwd);
  int bus_fwd = -1;
  if (enabled == 1) {
    if (bus_num == 0) {
      bus_fwd = hyundai_camera_bus;
    }
    if (bus_num == hyundai_camera_bus) {
      bus_fwd = 0;
    }
  }
  return bus_fwd;
}

static void forward_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
}

const safety_hooks forward_hooks = {
  .init = forward_init,
  .rx = forward_rx_hook,
  .tx = forward_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .ignition = default_ign_hook,
  .fwd = forward_fwd_hook,
};
