// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-

// arduino due defs
#define CONSOLE SERIAL_PORT_USBVIRTUAL
#define SERIAL2 SERIAL_PORT_HARDWARE_OPEN1
#define SERIAL3 SERIAL_PORT_HARDWARE_OPEN2

#include <ArduinoRS485.h>

#include "modbus.h"
#include "panic.h"
#include "crc16.h"

size_t no_rx;

// RS485Class serial2(HardwareSerial, pin_tx, pin_de, pin_re);
// tx = transmit pin ? only used to send breaks: ?, default PIN_SERIAL1_TX, or 1
// de = initially set low, taken high before transmit, taken low to recv: ?
// data_enable: default A6 re = initial set high, reset low, low on receive,
// high when not receive: receive_enable: default A5

RS485Class serial2(SERIAL2, -1, A6, A5);

char const hex[17] = "0123456789abcdef";
int to_hex(char outs[], size_t outs_len, uint8_t v) {
  if (outs_len < 2) {
    return -1;
  }
  outs[0] = hex[(v >> 4) & 0xf];
  outs[1] = hex[v & 0xf];
  return 2;
}

class LoggingIO : public FlushedIO {
  public:
    LoggingIO(FlushedIO &inner) : inner_(inner) {}

  public:
    virtual int write(uint8_t const arr[], size_t len) {
      dump_data("< ", arr, len);
      return inner_.write(arr, len);
    }

    virtual int write(uint8_t v) {
      dump_data("< ", v);
      return inner_.write(v);
    }

    virtual void flush_output(void) {
      inner_.flush_output();
    }

  public:
    virtual int in_waiting(void) {
      return inner_.in_waiting();
    }

    virtual int read(uint8_t arr[], size_t len) {
      int rc = inner_.read(arr, len);
      if (rc >= 0) {
        dump_data("> ", arr, rc);
      }
      return rc;
    }

    virtual int read(uint8_t &v) {
      int rc = inner_.read(v);
      if (rc >= -1) {
        dump_data("> ", v);
      }
      return rc;
    }

    virtual void flush_input(void) {
      inner_.flush_input();
    };

  private:
    void dump_data(char const *const prefix, uint8_t v) {
      char buf[2];
      CONSOLE.write(prefix);
      int n = to_hex(buf, sizeof(buf), v);
      CONSOLE.write(buf, n);
      CONSOLE.println();
    }

    void dump_data(char const *const prefix, uint8_t const arr[], size_t len) {
      char buf[2];
      CONSOLE.write(prefix);
      for (uint8_t const *i=arr, *const e=arr+len; i != e; ++i) {
        int n = to_hex(buf, sizeof(buf), *i);
        CONSOLE.write(buf, n);
        CONSOLE.write(' ');
      }
      CONSOLE.println();
    }


  private:
    FlushedIO &inner_;
};

class FlushedIOrs485: public FlushedIO {
  public:
    FlushedIOrs485(RS485Class &dev): dev_(dev) {}
    enum Mode {
      Unknown,
      Reading,
      Writing,
    };

  public:
    virtual int write(uint8_t const arr[], size_t len) {
      switch (mode_) {
        case Writing:
        break;
        case Unknown:
        case Reading:
          dev_.noReceive();
          dev_.beginTransmission();
          mode_ = Writing;
        break;
        default:
          panic0();
      }
      return dev_.write(arr, len);
    }
    virtual int write(uint8_t v) {
      switch (mode_) {
        case Writing:
        break;
        case Unknown:
        case Reading:
          dev_.noReceive();
          dev_.beginTransmission();
          mode_ = Writing;
        break;
        default:
          panic0();
      }
      return dev_.write(v);
    }
    virtual void flush_output(void) {
      switch (mode_) {
        case Writing:
          dev_.endTransmission();
          mode_ = Unknown;
        break;
        case Unknown:
        break;
        case Reading:
        break;
        default:
          panic0();
      }
    };

  public:
    virtual int in_waiting(void) {
      return dev_.available();
    }
    virtual int read(uint8_t arr[], size_t len) {
      switch (mode_) {
        case Writing:
          dev_.endTransmission();
          dev_.receive();
          mode_ = Reading;
        break;
        case Unknown:
          dev_.receive();
          mode_ = Reading;
        break;
        case Reading:
        break;
        default:
          panic0();
      }
      return dev_.readBytes(arr, len);
    }
    virtual int read(uint8_t &v) {
      switch (mode_) {
        case Writing:
          dev_.endTransmission();
          dev_.receive();
          mode_ = Reading;
        break;
        case Unknown:
          dev_.receive();
          mode_ = Reading;
        break;
        case Reading:
        break;
        default:
          panic0();
      }
      int rc = dev_.read();
      if (rc >= 0) {
        v = rc;
      }
      return rc;
    }
    virtual void flush_input(void) {
      switch (mode_) {
        case Writing:
          dev_.endTransmission();
          dev_.receive();
          mode_ = Reading;
        break;
        case Unknown:
          dev_.receive();
          mode_ = Reading;
        break;
        case Reading: {
          // drain the input buffer of contents.
          // but only what was there at time of call
          // don't drain any new ones..?
          //int n = dev_.available();
          //for (; n > 0; --n) {
          //  dev_.read();
          //}
          while ( dev_.available() > 0) {
            dev_.read();
          }
        }
        break;
        default:
          panic0();
      }
    };

  private:
    Mode mode_;
    RS485Class &dev_;
};

FlushedIOrs485 serial2_flushed(serial2);
LoggingIO serial2_logged(serial2_flushed);
modbus::rtu::ServerSocket sock(serial2_logged, 1750, 750, 3);

void setup() {
  // put your setup code here, to run once:

  SerialUSB.begin(115200);
  while (!SerialUSB)
    ;

  // set up serial2, serial3 and transfer bytes between
  Serial2.begin(115200, SERIAL_8E1);
  serial2.begin(115200, SERIAL_8E1, 100, 100);
  // Serial2.begin(115200, SERIAL_8N1);
  // serial2.begin(115200, SERIAL_8N1, 100, 100);
  serial2.receive();

  // pinMode(LED_BUILTIN, OUTPUT);

  no_rx = 0;
  SerialUSB.println("initialised..");
}

#define RX_BUF_SIZE 256
uint8_t rx_buf[RX_BUF_SIZE];
char mon_buf[RX_BUF_SIZE * 3];


int to_hex(char outs[], size_t outs_len, uint8_t ins[], size_t ins_len) {
  size_t o_idx = 0;
  for (size_t i_idx = 0; i_idx < ins_len; ++i_idx) {
    int n = to_hex(&outs[o_idx], outs_len - o_idx, ins[i_idx]);
    if (n < 0) {
      return n;
    }
    o_idx += n;
    if ((outs_len - o_idx) < 1) {
      break;
    }
    outs[o_idx] = ' ';
    o_idx += 1;
  }
  return o_idx;
}

modbus::RegisterValue regs[1000];



void loop() {
  // put your main code here, to run repeatedly:
  size_t n_msg = 0;
  modbus::MasterPdu in_msg;
  modbus::SlavePdu out_msg;

  while (sock.recv(in_msg)) {
    // do something with msg
    CONSOLE.print("message ");
    CONSOLE.print(n_msg);
    CONSOLE.println(" begin");

    switch (in_msg.opcode) {
      case modbus::MasterPdu::ReadRegMulti:
        CONSOLE.print("Got master::ReadRegMulti ");
        CONSOLE.print(in_msg.reg_multi.start);
        CONSOLE.print(' ');
        CONSOLE.println(in_msg.reg_multi.len);
        out_msg.opcode = modbus::SlavePdu::ReadRegMulti;
        for (size_t i = in_msg.reg_multi.start; i < in_msg.reg_multi.len; ++i) {
          out_msg.reg_multi.values[i] = regs[i];
        }
        out_msg.reg_multi.len = in_msg.reg_multi.len;
        out_msg.epilogue_len = in_msg.epilogue_len;
        memcpy(out_msg.epilogue, in_msg.epilogue, in_msg.epilogue_len);
        sock.send(move(out_msg));
    }

    CONSOLE.print("message ");
    CONSOLE.print(n_msg);
    CONSOLE.println(" done");
    n_msg += 1;
  }
}
