#!/usr/bin/env python3
"""Voice-guided bench test, PROPS OFF: arm with the radio, switch the radio off, check the flight
controller disarms / reports a radio failsafe. Read-only from the laptop's side: the pilot arms with
the arm switch; this script only watches (tools/bench/rc_stream.py on the Pi). Log: failsafe_check.log.
"""
import os
import subprocess
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, 'failsafe_check.log')
CMD = ['ssh', 'group7@10.43.0.42',
       'cd ~/Drone_Navigation/robot && docker compose run --rm mission python3 /logs/rc_stream.py']
st = {'age': 999.0, 'ch': [0] * 8, 'mode': '-', 'armed': False}
texts = []
log = open(LOG, 'a')


def note(s):
    line = time.strftime('%H:%M:%S ') + s
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


def say(t):
    note('SAY: ' + t)
    subprocess.run(['spd-say', '-r', '-10', '-w', t], stderr=subprocess.DEVNULL)


def reader(p):
    for line in p.stdout:
        parts = line.split()
        if parts and parts[0] == 'RC' and len(parts) >= 12:
            armed = parts[11] == '1'
            if armed != st['armed']:
                note(f'ARMED={armed} mode {parts[10]}')
            st.update(age=float(parts[1]), ch=[int(x) for x in parts[2:10]], mode=parts[10], armed=armed)
        elif parts and parts[0] == 'TEXT':
            texts.append(line.strip())
            note(line.strip())


def wait(cond, secs):
    end = time.time() + secs
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.1)
    return cond()


def main():
    p = subprocess.Popen(CMD, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    threading.Thread(target=reader, args=(p,), daemon=True).start()
    if not wait(lambda: st['age'] < 0.5, 60):
        say('No radio data. Turn the radio on, then start again.')
        return
    say('Radio failsafe test. Props must be off. Left stick fully down.')
    time.sleep(2)
    say('Now flip the arm switch to arm.')
    if not wait(lambda: st['armed'], 60):
        say('It did not arm. Flip the arm switch back off.')
        note('did not arm; FC messages: ' + (' | '.join(texts[-5:]) or 'none'))
        return
    say('Armed. Now switch the radio off.')
    if not wait(lambda: st['age'] > 1.0, 60):
        say('Radio still on. Flip the arm switch off. Test stopped.')
        return
    t_off = time.time()
    note('radio data stopped')
    disarmed = wait(lambda: not st['armed'], 20)
    dt = time.time() - t_off
    if disarmed:
        note(f'PASS: FC disarmed {dt:.1f} s after the radio went off')
        say(f'Pass. The drone disarmed after {dt:.0f} seconds.')
    else:
        note('FAIL: still armed 20 s after the radio went off')
        say('Fail. Still armed. Flip the arm switch off, then switch the radio on, and flip it off again.')
    say('Flip the arm switch to off. Then switch the radio back on.')
    wait(lambda: st['age'] < 0.5, 120)
    time.sleep(3)
    note(f'radio back; armed={st["armed"]}; FC messages during test: ' + (' | '.join(texts) or 'none'))
    if st['armed']:
        say('Still armed. Flip the arm switch off now.')
        wait(lambda: not st['armed'], 60)
    say('Test finished.')
    p.terminate()


if __name__ == '__main__':
    main()
