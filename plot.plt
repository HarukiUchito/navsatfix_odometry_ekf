set size ratio -1
plot \
    "dump/odom_measurements.txt" using 1:2 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "Wheel Odometry", \
    "dump/navsatfix_measurements.txt" using 1:2 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "GNSS", \
    "dump/ekf.txt" using 1:2 with linespoints pointtype 7 pointsize 0.2 linewidth 0.2 title "EKF"
pause 0.2
reread