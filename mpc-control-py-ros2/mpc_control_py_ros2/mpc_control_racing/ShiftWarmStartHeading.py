import numpy as np

def shift_warm_heading(state_heading, Heading_warm):
    print_debug = False
    if print_debug: print("heading:", state_heading, ", warm:", Heading_warm)

    modify = False
    for i in range(10):
        heading_err = state_heading - Heading_warm[0]
        warm_correction = 0
        if heading_err < -np.pi: warm_correction = -1
        elif heading_err > np.pi: warm_correction = +1

        if warm_correction != 0:
            Heading_warm += warm_correction * 2.0 * np.pi
            modify = True
            print("Warm-start heading +=", warm_correction, end='')
            if print_debug: print(" =>", Heading_warm)
            print()
        else:
            break

    return modify
