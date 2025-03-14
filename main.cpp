#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib> // per system()

// Controllore PID per il feedback
class PIDController {
public:
    double Kp, Ki, Kd;
    double previous_error;
    double integral;
    
    PIDController(double Kp_, double Ki_, double Kd_)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), previous_error(0.0), integral(0.0) {}

    // Aggiorna il controllo in base all'errore e al passo temporale dt
    double update(double error, double dt) {
        integral += error * dt;
        double derivative = (error - previous_error) / dt;
        previous_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
};

// Classe che simula la dinamica di una massa su piano inclinato
class MassOnIncline {
public:
    double M;      // massa [kg]
    double theta;  // angolo del piano (in radianti)
    double g;      // accelerazione di gravità
    double x;      // posizione
    double v;      // velocità

    MassOnIncline(double M_, double theta_, double g_ = 9.81)
        : M(M_), theta(theta_), g(g_), x(0.0), v(0.0) {}

    // Aggiorna lo stato del sistema in base alla forza F e al passo dt
    void update(double F, double dt) {
        double a = (F / M) - g * sin(theta);
        v += a * dt;
        x += v * dt;
    }
};

// Classe che simula il circuito RC con diodo
class DiodeRC {
public:
    double R;      // resistenza [Ohm]
    double C;      // capacità [Farad]
    double V_d;    // caduta di tensione del diodo [V]
    double v_C;    // tensione sul condensatore

    DiodeRC(double R_, double C_, double V_d_)
        : R(R_), C(C_), V_d(V_d_), v_C(0.0) {}

    // Simula un passo temporale dt in funzione della tensione in ingresso v_in
    void update(double v_in, double dt) {
        // Il diodo conduce se la differenza (v_in - v_C) supera la caduta V_d
        if (v_in - v_C > V_d) {
            // Se il diodo conduce, il condensatore si carica
            double dv = (v_in - V_d - v_C) / (R * C) * dt;
            v_C += dv;
        }
        // Altrimenti, il condensatore mantiene la sua carica (o si scarica lentamente)
    }
};

int main() {
    // Parametri generali della simulazione
    double dt = 0.001;   // passo temporale (s)
    double t_end = 5.0;  // durata simulazione (s)
    int steps = static_cast<int>(t_end / dt);

    // --- Simulazione 1: Controllo dell'equilibrio su piano inclinato ---
    std::cout << "Simulazione 1: Controllo dell'equilibrio su piano inclinato\n";

    // Apri file CSV per salvare i dati (colonne: t, x, v)
    std::ofstream sim1_file("simulation1.csv");
    sim1_file << "t,x,v\n";

    // Parametri del sistema
    double M = 1.0;                // massa in kg
    double theta = M_PI / 6;       // 30° in radianti
    MassOnIncline system(M, theta);
    system.x = 0.1;                // deviazione iniziale

    // Controllore PID per il feedback (Kp, Ki, Kd)
    PIDController pid1(10.0, 1.0, 0.5);
    double target_x = 0.0;         // posizione di riferimento

    // Loop di simulazione per il sistema della massa
    for (int i = 0; i < steps; i++) {
        double t = i * dt;
        // Feedforward: annulla la componente gravitazionale
        double F_ff = M * system.g * sin(theta);
        // Feedback: corregge l'errore di posizione
        double error = target_x - system.x;
        double F_fb = pid1.update(error, dt);
        double F = F_ff + F_fb;
        
        system.update(F, dt);
        
        // Salva i dati ad ogni passo
        sim1_file << t << "," << system.x << "," << system.v << "\n";
    }
    sim1_file.close();

    // --- Simulazione 2: Controllo del circuito RC con diodo ---
    std::cout << "Simulazione 2: Controllo del circuito RC con diodo\n";

    // Apri file CSV per salvare i dati (colonne: t, v_C, v_in, u)
    std::ofstream sim2_file("simulation2.csv");
    sim2_file << "t,v_C,v_in,u\n";

    // Parametri del circuito
    double R = 1000.0;       // resistenza (Ohm)
    double C = 1e-6;         // capacità (Farad)
    double V_d = 0.7;        // caduta del diodo (V)
    DiodeRC circuit(R, C, V_d);
    
    // Controllore PID per regolare la tensione sul condensatore
    PIDController pid2(50.0, 0.1, 0.1);
    double target_vC = 5.0;  // tensione di riferimento (V)
    
    // Feedforward per il generatore sinusoidale
    double u_ff = target_vC + V_d;
    double u = u_ff;
    double omega = 2 * M_PI * 50;  // frequenza del generatore (50 Hz)

    // Loop di simulazione per il circuito RC
    for (int i = 0; i < steps; i++) {
        double t = i * dt;
        double sin_val = sin(omega * t);
        
        // Calcola l'errore per il controllo del condensatore
        double error = target_vC - circuit.v_C;
        double u_fb = pid2.update(error, dt);
        u = u_ff + u_fb;
        
        double v_in = u * sin_val;
        circuit.update(v_in, dt);
        
        // Salva i dati ad ogni passo
        sim2_file << t << "," << circuit.v_C << "," << v_in << "," << u << "\n";
    }
    sim2_file.close();

    // --- Generazione dello script per gnuplot ---
    std::ofstream gp("plot.gp");
    gp << "set datafile separator ','\n";

    // Grafico per la simulazione 1 (massa su piano inclinato)
    gp << "set terminal pngcairo size 800,600\n";
    gp << "set output 'simulation1.png'\n";
    gp << "set title 'Simulazione 1: Controllo Equilibrio su Piano Inclinato'\n";
    gp << "set xlabel 'Tempo (s)'\n";
    gp << "set ylabel 'Posizione (m) e Velocita'\\'' (m/s)'\n";
    gp << "plot 'simulation1.csv' using 1:2 title 'Posizione (x)' with lines, \\\n";
    gp << "     'simulation1.csv' using 1:3 title 'Velocita (v)' with lines\n";

    // Grafico per la simulazione 2 (circuito RC con diodo)
    gp << "set output 'simulation2.png'\n";
    gp << "set title 'Simulazione 2: Controllo Circuito RC con Diodo'\n";
    gp << "set xlabel 'Tempo (s)'\n";
    gp << "set ylabel 'Tensioni (V)'\n";
    gp << "plot 'simulation2.csv' using 1:2 title 'Tensione sul condensatore (v_C)' with lines, \\\n";
    gp << "     'simulation2.csv' using 1:3 title 'Tensione ingresso (v_in)' with lines, \\\n";
    gp << "     'simulation2.csv' using 1:4 title 'Ampiezza (u)' with lines\n";
    gp.close();

    // Esecuzione dello script gnuplot per generare i grafici
    int ret = std::system("gnuplot plot.gp");
    if(ret == 0)
        std::cout << "\nGrafici generati: simulation1.png e simulation2.png\n";
    else
        std::cout << "\nErrore nell'esecuzione di gnuplot. Verifica di avere gnuplot installato.\n";

    return 0;
}
