#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib> // per std::system

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
        // a = (F_control/M) - g*sin(theta)
        double a = (F / M) - g * sin(theta);
        v += a * dt;
        x += v * dt;
    }
};

int main() {
    // Parametri della simulazione
    double dt = 0.001;       // passo temporale (s)
    double t_end = 5.0;      // durata simulazione (s)
    int steps = static_cast<int>(t_end / dt);

    // --- Simulazione: Controllo dell'equilibrio su piano inclinato ---
    std::cout << "Simulazione: Controllo dell'equilibrio su piano inclinato\n";

    // Apri file CSV per salvare i dati (colonne: t, x, v)
    std::ofstream sim_file("simulation1.csv");
    sim_file << "t,x,v\n";

    // Parametri del sistema
    double M = 1.0;                // massa in kg
    double theta = M_PI / 6;       // 30° in radianti
    MassOnIncline system(M, theta);
    system.x = 0.1;                // piccola deviazione iniziale per vedere l'azione del PID

    // Controllore PID per il feedback (Kp, Ki, Kd)
    PIDController pid(10.0, 1.0, 0.5);
    double target_x = 0.0;         // posizione di riferimento

    // Loop di simulazione per il sistema della massa
    for (int i = 0; i < steps; i++) {
        double t = i * dt;
        // Feedforward: forza necessaria a bilanciare la componente gravitazionale
        double F_ff = M * system.g * sin(theta);
        // Feedback: corregge l'errore di posizione
        double error = target_x - system.x;
        double F_fb = pid.update(error, dt);
        double F = F_ff + F_fb;
        
        system.update(F, dt);
        
        // Salva i dati ad ogni passo
        sim_file << t << "," << system.x << "," << system.v << "\n";
    }
    sim_file.close();

    // --- Generazione del phase portrait tramite gnuplot ---
    // Il phase portrait mostra la velocità (v) in funzione della posizione (x).
    std::ofstream gp("phase_portrait.gp");
    gp << "set datafile separator ','\n";
    gp << "set terminal pngcairo size 800,600\n";
    gp << "set output 'phase_portrait.png'\n";
    gp << "set title 'Phase Portrait: Massa su Piano Inclinato'\n";
    gp << "set xlabel 'Posizione (x) [m]'\n";
    gp << "set ylabel 'Velocita (v) [m/s]'\n";
    gp << "plot 'simulation1.csv' using 2:3 title 'Phase Portrait' with linespoints\n";
    gp.close();

    // Esecuzione dello script gnuplot per generare il phase portrait
    int ret = std::system("gnuplot phase_portrait.gp");
    if(ret == 0)
        std::cout << "\nPhase portrait generato: phase_portrait.png\n";
    else
        std::cout << "\nErrore nell'esecuzione di gnuplot per il phase portrait. Verifica l'installazione di gnuplot.\n";

    return 0;
}
