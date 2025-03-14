set datafile separator ','
set terminal pngcairo size 800,600
set output 'phase_portrait.png'
set title 'Phase Portrait: Massa su Piano Inclinato'
set xlabel 'Posizione (x) [m]'
set ylabel 'Velocita (v) [m/s]'
plot 'simulation1.csv' using 2:3 title 'Phase Portrait' with linespoints
