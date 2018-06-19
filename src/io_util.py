import sys, termios, tty, os, time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def wait_for_press():
    pressed = False
    char = 0

    while not pressed:
        char = getch()
        if char != 0:
            pressed = True
        else:
            time.sleep(0.2)
    return char

    ## ADD LOOP WITH DELAY
    #time.sleep(amt of time)
