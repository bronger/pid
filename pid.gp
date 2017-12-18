set term png
set grid
set output "/home/bronger/public_html/1.png"
plot "pid.dat" using 1:2 with lines
