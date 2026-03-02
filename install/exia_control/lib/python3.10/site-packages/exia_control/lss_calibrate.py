#!/usr/bin/env python3
import sys
import time
import termios
import tty
import select

SERVO_NAMES = {1: 'THROTTLE', 2: 'BRAKE', 3: 'GEAR'}
STEP_SMALL = 10
STEP_BIG = 50


def main():
    try:
        import serial
    except ImportError:
        print('pyserial not installed')
        return

    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/lss_controller'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    try:
        conn = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        time.sleep(0.5)
        conn.reset_input_buffer()
    except Exception as e:
        print(f'Failed to open {port}: {e}')
        return

    print('LSS Servo Calibration')
    print('---------------------')
    print('  1/2/3     : select servo (throttle/brake/gear)')
    print('  UP/DOWN   : adjust position +/-10')
    print('  PgUp/PgDn : adjust position +/-50')
    print('  L         : limp (disable) current servo')
    print('  H         : hold current position')
    print('  Q         : query status')
    print('  Ctrl+C    : quit')
    print()

    active = 1
    positions = {1: 0, 2: 0, 3: 0}

    for sid in (1, 2, 3):
        conn.write(f'#{sid}SD600\r'.encode())
        conn.flush()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    def send(sid, pos):
        conn.write(f'#{sid}D{pos}\r'.encode())
        conn.flush()

    def print_status():
        sys.stdout.write('\r\033[K')
        parts = []
        for sid in (1, 2, 3):
            marker = '>' if sid == active else ' '
            parts.append(f'{marker}{SERVO_NAMES[sid]}({sid}):{positions[sid]:+5d}')
        sys.stdout.write('  '.join(parts))
        sys.stdout.flush()

    print_status()

    try:
        while True:
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            ch = sys.stdin.read(1)

            if ch == '\x1b':
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    sys.stdin.read(1)
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        code = sys.stdin.read(1)
                        if code == 'A':
                            positions[active] += STEP_SMALL
                            send(active, positions[active])
                        elif code == 'B':
                            positions[active] -= STEP_SMALL
                            send(active, positions[active])
                        elif code == '5':
                            if select.select([sys.stdin], [], [], 0.05)[0]:
                                sys.stdin.read(1)
                            positions[active] += STEP_BIG
                            send(active, positions[active])
                        elif code == '6':
                            if select.select([sys.stdin], [], [], 0.05)[0]:
                                sys.stdin.read(1)
                            positions[active] -= STEP_BIG
                            send(active, positions[active])
                print_status()
                continue

            if ch in ('1', '2', '3'):
                active = int(ch)
            elif ch.lower() == 'l':
                conn.write(f'#{active}L\r'.encode())
                conn.flush()
            elif ch.lower() == 'h':
                conn.write(f'#{active}H\r'.encode())
                conn.flush()
            elif ch.lower() == 'q':
                conn.reset_input_buffer()
                conn.write(f'#{active}QD\r'.encode())
                conn.flush()
                time.sleep(0.1)
                resp = conn.read(conn.in_waiting).decode(errors='ignore').strip()
                sys.stdout.write(f'\n  Query #{active}: {resp}\n')
            elif ch == '\x03':
                break

            print_status()

    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        for sid in (1, 2, 3):
            conn.write(f'#{sid}L\r'.encode())
        conn.flush()
        conn.close()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print()
        print()
        print('Final positions:')
        for sid in (1, 2, 3):
            print(f'  {SERVO_NAMES[sid]} (ID {sid}): {positions[sid]}')


if __name__ == '__main__':
    main()
