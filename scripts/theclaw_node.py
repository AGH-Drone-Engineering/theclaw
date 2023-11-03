#!/usr/bin/env python

import theclaw.main

if __name__ == '__main__':
    try:
        theclaw.main.run()
    except rospy.ROSInterruptException:
        pass
