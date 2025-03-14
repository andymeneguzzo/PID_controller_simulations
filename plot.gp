set datafile separator ','
set terminal pngcairo size 800,600
set output 'simulation1.png'
set title 'Simulazione 1: Controllo Equilibrio su Piano Inclinato'
set xlabel 'Tempo (s)'
set ylabel "Posizione (m) e Velocita (m/s)"
plot 'simulation1.csv' using 1:2 title 'Posizione (x)' with lines, \
     'simulation1.csv' using 1:3 title 'Velocita (v)' with lines
set output 'simulation2.png'
set title 'Simulazione 2: Controllo Circuito RC con Diodo'
set xlabel 'Tempo (s)'
set ylabel 'Tensioni (V)'
plot 'simulation2.csv' using 1:2 title 'Tensione sul condensatore (v_C)' with lines, \
     'simulation2.csv' using 1:3 title 'Tensione ingresso (v_in)' with lines, \
     'simulation2.csv' using 1:4 title 'Ampiezza (u)' with lines
