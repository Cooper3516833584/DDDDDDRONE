"""Compile the actual wireless C parser with host-only UART/USB stubs.

No serial ports or flight-controller APIs are opened. A native C compiler is
required; --source - reads candidate Wireless_Com.c source from stdin.
"""
import argparse
from pathlib import Path
import re
import subprocess
import sys
import tempfile


STUBS = r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8;
static unsigned received;
static const u8 *expected;
static unsigned expected_len;
void DrvUart2SendBuf(u8 *data, unsigned len) { (void)data; (void)len; }
void cdc_acm_data_send(uint8_t *data, uint32_t len) {
  if (len != expected_len + 5 || data[0] != 0xAA || data[1] != 0x55 ||
      data[2] != expected_len + 1 || data[3] != 7 ||
      memcmp(data + 4, expected, expected_len)) {
    fprintf(stderr, "incorrect forwarded frame at index %u\n", received);
    exit(2);
  }
  u8 sum = 0;
  for (unsigned i = 0; i < len - 1; ++i) sum += data[i];
  if (sum != data[len - 1]) { fprintf(stderr, "USB checksum mismatch\n"); exit(3); }
  ++received;
}
'''

CASES = r'''
static void frame(const u8 *payload, unsigned len) {
  expected = payload;
  expected_len = len;
  Uart_Wireless_GetOneByte(0xBB);
  Uart_Wireless_GetOneByte(0x33);
  Uart_Wireless_GetOneByte((u8)len);
  for (unsigned i = 0; i < len; ++i) Uart_Wireless_GetOneByte(payload[i]);
}
int main(void) {
  const u8 payload[] = {0xD3,0x91,0xBB,0x33,0,0x1D,0x0F};
  for (unsigned i = 0; i < 10; ++i) frame(payload, sizeof payload);
  if (received != 10) {
    fprintf(stderr, "contiguous frames: expected 10, received %u\n", received);
    return 4;
  }
  const u8 one[] = {0};
  frame(one, sizeof one);
  u8 max_fleet_frame[239];
  for (unsigned i = 0; i < sizeof max_fleet_frame; ++i) max_fleet_frame[i] = (u8)i;
  frame(max_fleet_frame, sizeof max_fleet_frame);
  if (received != 12) { fprintf(stderr, "payload boundary frame lost\n"); return 5; }
  Uart_Wireless_GetOneByte(0xBB);
  Uart_Wireless_GetOneByte(0x33);
  Uart_Wireless_GetOneByte(0);
  frame(payload, sizeof payload);
  if (received != 13) { fprintf(stderr, "zero-length envelope consumed next header\n"); return 6; }
  puts("PASS: 13 frames; contiguous packets, embedded headers, 1/239-byte payloads, zero-length recovery, USB frame/checksum");
  return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', default=str(Path(__file__).resolve().parents[2]/'FcSrc/Wireless_Com.c'))
    parser.add_argument('--cc', default='cc')
    args = parser.parse_args()
    source = sys.stdin.read() if args.source == '-' else Path(args.source).read_text(encoding='utf-8')
    # Only replace board headers; compile the complete production C body.
    source = re.sub(r'^#include "(?:Wireless_Com|Drv_Uart)\.h"\s*$', '', source, flags=re.MULTILINE)
    with tempfile.TemporaryDirectory(prefix='wireless-parser-') as directory:
        executable = str(Path(directory)/'wireless-parser-check')
        subprocess.run([args.cc, '-std=c99', '-Wall', '-Wextra', '-Werror', '-x', 'c', '-', '-o', executable],
                       input=STUBS+'\n'+source+'\n'+CASES, text=True, check=True)
        return subprocess.run([executable], check=False).returncode


if __name__ == '__main__':
    raise SystemExit(main())
