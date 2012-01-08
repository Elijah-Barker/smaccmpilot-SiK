#!/usr/bin/env python
# reflect input bytes to output, printing as it goes

import serial, sys, optparse, time

parser = optparse.OptionParser("pattern")
parser.add_option("--baudrate", type='int', default=115200, help='baud rate')
parser.add_option("--delay", type='float', default=0.0, help='delay between lines')
parser.add_option("--pattern", type='str', default='0123456789', help='pattern to send')
parser.add_option("--echo", action='store_true', default=False, help='echo any bytes received')
parser.add_option("--crlf", action='store_true', default=False, help='add crlf')
parser.add_option("--counter", action='store_true', default=False, help='add counter')


opts, args = parser.parse_args()

if len(args) != 1:
    print("usage: pattern.py <DEVICE>")
    sys.exit(1)

device = args[0]

port = serial.Serial(device, opts.baudrate, timeout=0,
                     dsrdtr=False, rtscts=False, xonxoff=False)

counter = 0
while True:
    try:
        buf = opts.pattern[:]
        if opts.counter:
            buf += "%02u" % (counter % 100)
        if opts.crlf:
            buf += '\r\n'
        port.write(buf)
        if opts.echo:
            try:
                buf = port.read()
                sys.stdout.write(buf)
            except Exception:
                pass
        if opts.delay > 0.0:
            time.sleep(opts.delay)
        counter += 1
    except KeyboardInterrupt:
        sys.exit(0)
