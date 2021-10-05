set xrange [-1:1]
set yrange [-0.8:0.8]
set object 1 rect from -0.96,-0.57 to 0.96,0.57 lw 5 fs empty border lc rgb '#000000'
plot "trajectory.dat" with vectors title "robot trajectory"
