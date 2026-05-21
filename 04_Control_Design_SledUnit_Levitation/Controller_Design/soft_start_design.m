%#########################################################################%
%#                                                                       #%
%#  DATEI:  soft_start_design.m                                          #%
%#                                                                       #%
%#  BESCHREIBUNG:                                                        #%
%#  Entwurf einer Sanftanlauf-Sequenz für das 5-DOF-Magnetlager.         #%
%#  Das Skript berechnet sichere Rampenparameter auf Basis des           #%
%#  geschlossenen Regelkreises, entwirft eine quintic S-Kurven-          #%
%#  Referenztrajektorie und validiert die Sequenz durch Simulation.      #%
%#                                                                       #%
%#  PHASEN:                                                              #%
%#    Phase 0 — Vormagnetisierung  (offene Schlaufe, Ströme auframpieren)#%
%#    Phase 1 — Einrasten          (Regler aktivieren, Fehler ≈ 0)       #%
%#    Phase 2 — Rampe              (Sollwert glatt zum Nominallufspalt)  #%
%#    Phase 3 — Normalbetrieb      (voller LQR-Betrieb)                  #%
%#                                                                       #%
%#  ABSCHNITTE:                                                          #%
%#    SS1 — Reglerentwurfsparameter laden                                #%
%#    SS2 — Sicheren Linearisierungsbereich schätzen                     #%
%#    SS3 — Maximale Rampenrate berechnen                                #%
%#    SS4 — Quintic S-Kurven-Trajektorie entwerfen                       #%
%#    SS5 — Simulationsvalidierung                                       #%
%#    SS6 — Parameter-Export für Simulink                                #%
%#                                                                       #%
%#########################################################################%

clc; clear; close all;

%% SS1: Reglerentwurfsparameter laden
%--------------------------------------------------------------------------
% Alle für den Soft-Start benötigten Größen stammen aus dem bereits
% abgeschlossenen Zustandsraum-Reglerentwurf. Es werden keine neuen
% Regler entworfen — nur Trajektorienparameter berechnet.
%--------------------------------------------------------------------------

% Pfad ergänzen und Parameterdateien laden
addpath('..\00_Gemeinsame_Bibliothek\')
run("setup_machine_model\setup_structure_params");

load(".\Parameters\ControllerParams");   % ssc_params: K_aug, K_aw, dim_s, ...
load(".\Parameters\ObserverParams");     % obs_params: A_sys, B_sys, C_obs, L_obs

% Systemmatrizen und Reglerparameter extrahieren
A      = obs_params.A_sys;           % 10×10 Systemmatrix
B      = obs_params.B_sys;           % 10×8  Eingangsmatrix
C_obs  = obs_params.C_obs;           % 8×10  Messmatrix
L_obs  = obs_params.L_obs;           % 10×8  Kalman-Verstärkung

K_aug  = ssc_params.K_aug;           % 8×15  Erweiterter LQR-Regler
K_aw   = ssc_params.K_aw;            % 5×8   Anti-Windup-Verstärkung
I0_vec = ssc_params.nom_curvec;      % 1×8   Arbeitspunktströme [A]
x0_gap = ssc_params.nom_airgap*1e-3; % Nominaler Luftspalt [m]

dim_s  = ssc_params.dim_s;           % Anzahl der Lage-DOFs = 5
dim_u  = ssc_params.dim_u;           % Anzahl der Eingänge  = 8
dim_z  = dim_s;                      % Anzahl der Integratoren = 5

% Erweitertes Systemtripel für Polberechnung rekonstruieren
Ks     = A(dim_s+1:end, 1:dim_s);    % 5×5  Positionssteifigkeitsmatrix
Ki_mat = B(dim_s+1:end, :);          % 5×8  Strom-Kraft-Matrix

A_aug  = [A,             zeros(2*dim_s, dim_z);
          eye(dim_z),    zeros(dim_z, dim_s), zeros(dim_z, dim_z)];
B_aug  = [B; zeros(dim_z, dim_u)];

fprintf('=== SS1: Parameter geladen ===\n');
fprintf('  Nominaler Luftspalt:  %.2f mm\n', x0_gap*1e3);
fprintf('  Systemdimension:      %d Zustände (%d DOFs + %d Integratoren)\n', ...
        2*dim_s+dim_z, 2*dim_s, dim_z);

%% SS2: Sicheren Linearisierungsbereich schätzen
%--------------------------------------------------------------------------
% Die Kraft-Lücken-Beziehung F = Km·i²/x² wird um x0 taylorentwickelt:
%
%   F(x0+δx) ≈ F0 · [1 - 2(δx/x0) + 3(δx/x0)² - ...]
%
% Der quadratische Term 3(δx/x0)² ist der führende Linearisierungsfehler.
% Wir fordern, dass dieser Fehler unter epsilon_max bleibt:
%
%   3·(δx/x0)² < epsilon_max
%   → δx_safe < x0 · sqrt(epsilon_max / 3)
%
% Hinweis: Dies ist eine konservative Schätzung auf Basis der Kraftnichtlinearität.
% Die tatsächliche ROA des Reglers kann etwas größer sein, da der Integralanteil
% und die Regelverstärkung Linearisierungsfehler partiell kompensieren.
%--------------------------------------------------------------------------

epsilon_max  = 0.15;   % Akzeptabler relativer Linearisierungsfehler (15%)

% Sicherer Abweichungsbereich aus Taylor-Entwicklung [m]
delta_x_safe = x0_gap * sqrt(epsilon_max / 3);

fprintf('\n=== SS2: Linearisierungsbereich ===\n');
fprintf('  Akzeptierter Fehler:     %.0f%%\n', epsilon_max*100);
fprintf('  Sicherer Bereich δx:    ±%.3f mm\n', delta_x_safe*1e3);
fprintf('  Als Anteil des Spalts:   ±%.1f%% von x0\n', delta_x_safe/x0_gap*100);

% Zur Information: Kraftfehler an der Grenze verifizieren
Km_val = ElectromagnetConfig.MagConst;
F0     = Km_val * mean(I0_vec)^2 / x0_gap^2;
F_at_safe = Km_val * mean(I0_vec)^2 / (x0_gap + delta_x_safe)^2;
linearized_at_safe = F0 * (1 - 2*delta_x_safe/x0_gap);
fprintf('  Tatsächlicher Kraftfehler an Grenze: %.1f%%\n', ...
        abs(F_at_safe - linearized_at_safe)/F0 * 100);

%% SS3: Maximale Rampenrate berechnen
%--------------------------------------------------------------------------
% Während der Rampe bewegt sich der Sollwert mit der Rate v_ramp [m/s].
% Das Tracking-System des geschlossenen Regelkreises erzeugt dabei einen
% stationären Schleppfehler:
%
%   δx_track ≈ v_ramp / ω_CL
%
% Damit δx_track < δx_safe bleibt:
%   v_ramp < δx_safe · ω_CL
%
% ω_CL wird durch den langsamsten geschlossenen Pol bestimmt (betragskleinster
% negativer Realteil), da dieser die dominante Zeitkonstante der Reaktion vorgibt.
%--------------------------------------------------------------------------

% Geschlossene Polstellen des erweiterten Reglers
eig_CL = eig(A_aug - B_aug * K_aug);
omega_CL = min(abs(real(eig_CL)));   % Effektive Regelbandbreite [rad/s]

fprintf('\n=== SS3: Rampenrate ===\n');
fprintf('  Langsamster CL-Pol (Realteil): %.2f rad/s\n', -min(abs(real(eig_CL))));
fprintf('  Effektive Bandbreite ω_CL:     %.2f rad/s\n', omega_CL);

% Maximale sichere Rampenrate [m/s]
v_ramp_max = delta_x_safe * omega_CL;

% Designwert: 40% der maximalen Rate für Sicherheitsreserve
safety_factor = 0.4;
v_ramp = safety_factor * v_ramp_max;

fprintf('  Maximale Rampenrate:    %.2f mm/s\n', v_ramp_max*1e3);
fprintf('  Gewählte Rampenrate:    %.2f mm/s  (Faktor %.1f)\n', ...
        v_ramp*1e3, safety_factor);
fprintf('  → Erwarteter Schleppfehler: %.3f mm (< %.3f mm)\n', ...
        v_ramp/omega_CL*1e3, delta_x_safe*1e3);

%% SS4: Quintic S-Kurven-Trajektorie entwerfen
%--------------------------------------------------------------------------
% Anforderungen an die Trajektorie:
%   r(0)  = δ_init    (Startet an der Initialabweichung)
%   r(T)  = 0         (Endet am Nominallufspalt, Abweichung = 0)
%   ṙ(0)  = ṙ(T) = 0  (Stetige Geschwindigkeit → kein Stellgrößensprung)
%   r̈(0)  = r̈(T) = 0  (Stetige Beschleunigung → kein Kraftsprung)
%
% Lösung: Quintic Hermite-Polynom (Grad 5)
%
%   r(τ) = δ_init · (1 - 10τ³ + 15τ⁴ - 6τ⁵)   mit τ = t/T ∈ [0,1]
%
% Vorteil gegenüber linearer Rampe oder kubischer S-Kurve:
%   - Null-Beschleunigung an den Endpunkten verhindert Kraftsprünge
%   - Maximale Geschwindigkeit tritt bei τ = 0.5 auf (vorhersehbar)
%   - Keine Überschwinger in der Trajektorie selbst
%
% Phasendauern:
%   t_precharge : Vormagnetisierung (offene Schlaufe)
%   t_capture   : Einrasten des Reglers, Beobachter konvergiert
%   t_ramp      : Sollwert-Rampe von δ_init auf 0
%   t_settle    : Finale Einschwingzeit nach der Rampe
%--------------------------------------------------------------------------

% --- Maximale zu bewältigende Initialabweichung ---
delta_init_max = 1.0e-3;   % 1,0 mm (konservative Auslegung, deckt 0,5 mm ab)

% --- Phasendauern ---
t_precharge = 0.100;                          % 100 ms: Ströme von 0 auf I0 auframpieren
t_capture   = 0.050;                          % 50 ms: Regler ein, Fehler ≈ 0, KF konvergiert
t_ramp      = delta_init_max / v_ramp;        % Rampenzeit für max. Initialabweichung [s]
t_settle    = 5 / omega_CL;                   % 5 Zeitkonstanten zum Einschwingen [s]

t_total = t_precharge + t_capture + t_ramp + t_settle;

fprintf('\n=== SS4: Trajektorienparameter ===\n');
fprintf('  Phase 0 — Vormagnetisierung:  %.0f ms\n', t_precharge*1e3);
fprintf('  Phase 1 — Einrasten:          %.0f ms\n', t_capture*1e3);
fprintf('  Phase 2 — Rampe:              %.0f ms  (für δ_init = %.1f mm)\n', ...
        t_ramp*1e3, delta_init_max*1e3);
fprintf('  Phase 3 — Normalbetrieb:      %.0f ms  (Einschwingen)\n', t_settle*1e3);
fprintf('  Gesamtdauer Sanftanlauf:      %.0f ms\n', t_total*1e3);

% --- Quintic S-Kurven-Funktion ---
% Eingabe:  tau ∈ [0,1] (normierte Zeit)
% Ausgabe:  Skalierungsfaktor h(τ) ∈ [0,1], wobei r(τ) = δ_init · h(τ)
quintic_h  = @(tau)  1 - 10*tau.^3 + 15*tau.^4 - 6*tau.^5;
quintic_dh = @(tau) (-30*tau.^2 + 60*tau.^3 - 30*tau.^4) / t_ramp;   % ṙ [1/s]

% --- Visualisierung der Trajektorie ---
t_vec  = linspace(0, t_ramp, 500);
tau    = t_vec / t_ramp;
r_vec  = delta_init_max * quintic_h(tau);    % Positionssollwert [m]
dr_vec = delta_init_max * quintic_dh(tau);   % Geschwindigkeitssollwert [m/s]

figure('Name', 'Sanftanlauf-Trajektorie', 'Position', [100 100 800 500]);

subplot(2,1,1);
plot(t_vec*1e3, r_vec*1e3, 'b-', 'LineWidth', 2);
hold on;
yline(delta_x_safe*1e3,  'r--', sprintf('ROA-Grenze +%.2f mm', delta_x_safe*1e3));
yline(-delta_x_safe*1e3, 'r--');
yline(0, 'k:', 'Sollwert (x_0)');
xlabel('Zeit [ms]'); ylabel('Abweichung δx [mm]');
title('Quintic S-Kurven-Referenztrajektorie (Rampenphase)');
legend('Referenztrajektorie r(t)', 'ROA-Grenze', '', 'Nominallufspalt', ...
       'Location', 'northeast');
grid on; ylim([-0.1, delta_init_max*1e3 * 1.1]);

subplot(2,1,2);
plot(t_vec*1e3, dr_vec*1e3, 'b-', 'LineWidth', 2);
hold on;
yline(-v_ramp*1e3, 'r--', sprintf('Max. Rate %.2f mm/s', v_ramp*1e3));
xlabel('Zeit [ms]'); ylabel('Rampenrate ṙ [mm/s]');
title('Rampengeschwindigkeit (Maximum bei τ = 0,5)');
grid on;

% Maximale Rampenrate verifizieren
v_max_actual = max(abs(dr_vec));
fprintf('\n  Maximale Rampengeschwindigkeit: %.2f mm/s (Limit: %.2f mm/s)\n', ...
        v_max_actual*1e3, v_ramp*1e3);
fprintf('  Maximaler Schleppfehler erwartet: %.3f mm\n', v_max_actual/omega_CL*1e3);
assert(v_max_actual <= v_ramp_max, ...
    'Rampenrate überschreitet sicheres Limit! t_ramp vergrößern.');

%% SS5: Simulationsvalidierung
%--------------------------------------------------------------------------
% Das vereinfachte lineare Modell (Zustandsrückführung + Beobachter) wird
% mit der entworfenen Referenztrajektorie angeregt, um zu verifizieren, dass:
%   a) Der Zustandsfehler e = x - x_ref die ROA-Grenze nicht überschreitet
%   b) Die Stellgrößen u innerhalb der Strombegrenzungen bleiben
%   c) Die Integratorzustände sauber initialisiert werden
%
% Anmerkung: Diese Simulation verwendet das linearisierte Modell und ist daher
% nur gültig, wenn die Abweichungen klein bleiben (genau das, was geprüft wird).
%--------------------------------------------------------------------------

fprintf('\n=== SS5: Simulationsvalidierung ===\n');

% Diskretisierungsschrittweite für ODE-Simulation
dt      = 1e-4;    % 100 µs (entspricht typischer Echtzeit-Abtastrate)
t_sim   = 0 : dt : (t_ramp + t_settle);
N_steps = length(t_sim);

% Zustandsvektoren initialisieren
% Startzustand: Abweichung δ_init in allen translatorischen DOFs (worst case)
x_plant = zeros(2*dim_s, 1);
x_plant(1) = delta_init_max;    % x-Abweichung: δ_init [m]
x_plant(2) = delta_init_max;    % y-Abweichung: δ_init [m]
% Rotatorische DOFs: kleiner Anfangsfehler (0,1 mrad)
x_plant(3) = 1e-4;   % θ [rad]
x_plant(4) = 1e-4;   % φ [rad]
x_plant(5) = 1e-4;   % ψ [rad]

x_hat = zeros(2*dim_s, 1);      % Beobachterzustand (startet am Nullpunkt)
z_int = zeros(dim_s, 1);        % Integratorzustände (auf 0 initialisiert)

% Ergebnisvektoren vorallokieren
x_hist  = zeros(2*dim_s, N_steps);
u_hist  = zeros(dim_u,   N_steps);
ref_hist = zeros(dim_s,  N_steps);

% Zustandsextraktionsmatrizen
K_x  = K_aug(:, 1:2*dim_s);            % 8×10: Zustandsrückführung
K_z  = K_aug(:, 2*dim_s+1:end);        % 8×5:  Integralrückführung
C_pos = [eye(dim_s), zeros(dim_s)];     % 5×10: Extraktion der Lagekoordinaten

% Strombegrenzungen (aus Spezifikation)
u_min = (I_min - mean(I0_vec)) * ones(dim_u, 1);   % Untere Grenze der Stromabweichung [A]
u_max = (I_max - mean(I0_vec)) * ones(dim_u, 1);   % Obere Grenze der Stromabweichung [A]

for k = 1:N_steps
    t_k = t_sim(k);

    % --- Referenztrajektorie für aktuellen Zeitschritt ---
    if t_k <= t_ramp
        tau_k   = t_k / t_ramp;
        ref_k   = [delta_init_max * quintic_h(tau_k); 0; 0; 0; 0];  % [x,y,θ,φ,ψ]
    else
        ref_k   = zeros(dim_s, 1);   % Am Normallufspalt angekommen
    end
    ref_hist(:, k) = ref_k;

    % --- Regelgesetz: u = -K_x*(x̂ - r) - K_z*z ---
    % Fehler des Beobachters relativ zur Referenz (nicht relativ zum Ursprung)
    x_hat_err = x_hat - [ref_k; zeros(dim_s,1)];   % 10×1 Zustandsfehler
    u_cmd = -K_x * x_hat_err - K_z * z_int;         % 8×1 Befehlsstrom-Abweichung

    % Strombegrenzung anwenden
    u_sat  = min(max(u_cmd, u_min), u_max);          % Gesättigter Stellwert
    delta_u = u_sat - u_cmd;                          % Sättigungsfehler für Anti-Windup

    % --- Anlage (linearisiert, Euler-Vorwärtsintegration) ---
    x_plant = x_plant + dt * (A * x_plant + B * u_sat);

    % --- Kalman-Beobachter ---
    y_meas  = C_obs * x_plant;                        % Luftspaltsensor-Ausgaben (8×1)
    y_hat   = C_obs * x_hat;                          % Beobachter-Prädiktion
    x_hat   = x_hat + dt * (A * x_hat + B * u_sat + L_obs * (y_meas - y_hat));

    % --- Integratordynamik mit Anti-Windup ---
    % ż = q̂ - r + K_aw * (u_sat - u_cmd)
    z_int   = z_int + dt * (C_pos * x_hat_err + K_aw * delta_u);

    % --- Ergebnisse speichern ---
    x_hist(:, k)  = x_plant;
    u_hist(:, k)  = u_sat;
end

% --- Ergebnisplots ---
figure('Name', 'Sanftanlauf-Simulation', 'Position', [100 100 900 700]);

subplot(3,1,1);
plot(t_sim*1e3, x_hist(1:2,:)*1e3, 'LineWidth', 1.5);
hold on;
plot(t_sim*1e3, ref_hist(1,:)*1e3, 'k--', 'LineWidth', 1.5);
yline(delta_x_safe*1e3,  'r:', sprintf('+%.2f mm ROA', delta_x_safe*1e3));
yline(-delta_x_safe*1e3, 'r:');
xlabel('Zeit [ms]'); ylabel('Position [mm]');
title('Translatorische Abweichungen (x, y) mit Referenz');
legend({'x_{plant}', 'y_{plant}', 'Referenz r(t)', 'ROA-Grenze'}, 'Location', 'northeast');
grid on;

subplot(3,1,2);
plot(t_sim*1e3, x_hist(3:5,:)*1e3, 'LineWidth', 1.5);
xlabel('Zeit [ms]'); ylabel('Winkel [mrad]');
title('Rotatorische Abweichungen (θ, φ, ψ)');
legend({'θ (Roll)', 'φ (Nick)', 'ψ (Gier)'}, 'Location', 'northeast');
grid on;

subplot(3,1,3);
plot(t_sim*1e3, (u_hist + I0_vec')*1e0, 'LineWidth', 1.0);
hold on;
yline(I_max, 'r--', sprintf('I_{max} = %.0f A', I_max));
yline(I_min, 'b--', sprintf('I_{min} = %.0f A', I_min));
xlabel('Zeit [ms]'); ylabel('Strom [A]');
title('Magnetströme (absolut = I_0 + u)');
grid on;

% --- Quantitative Auswertung ---
max_dev_x = max(abs(x_hist(1,:) - ref_hist(1,:))) * 1e3;
max_dev_y = max(abs(x_hist(2,:) - ref_hist(2,:))) * 1e3;
max_current = max(max(abs(u_hist) + I0_vec'));
ss_error  = max(abs(x_hist(1:dim_s, end))) * 1e3;

fprintf('  Max. Schleppfehler x:   %.3f mm  (Grenze: %.3f mm)\n', ...
        max_dev_x, delta_x_safe*1e3);
fprintf('  Max. Schleppfehler y:   %.3f mm\n', max_dev_y);
fprintf('  Max. Strom absolut:     %.2f A   (Grenze: %.0f A)\n', max_current, I_max);
fprintf('  Bleibende Regelabw.:    %.4f mm\n', ss_error);

if max_dev_x < delta_x_safe*1e3 && max_current < I_max
    fprintf('  [OK] Sanftanlauf-Trajektorie validiert.\n');
else
    fprintf('  [WARNUNG] Parameter anpassen: t_ramp vergrößern oder safety_factor reduzieren.\n');
end

%% SS6: Parameter-Export für Simulink
%--------------------------------------------------------------------------
% Die Soft-Start-Parameter werden in einer .mat-Datei gespeichert, die
% direkt als Workspace-Import in das Simulink-Modell eingebunden werden kann.
%
% Simulink-Implementierung (empfohlene Struktur):
%
%   ┌──────────────┐    r(t)    ┌──────────────┐    u_cmd   ┌──────────┐
%   │  Soft-Start  │──────────▶ |  LQR-Regler  │──────────▶│  Anlage  │
%   │  Generator   │            │  + Integrator│            │(Simscape)│
%   └──────────────┘            └──────────────┘            └──────────┘
%          ▲                         ▲                         │
%          │ Phase-Signal            │ x̂                       │ y_sens
%          │                   ┌─────┴────┐                    │
%          └───────────────────│  Kalman  │◀──────────────────┘
%                              └──────────┘
%
% Zustandsmaschine im Soft-Start-Block:
%
%   Phase 0 (t < t_precharge):
%     → I_cmd = I0_vec * (t / t_precharge)    [Strominit-Rampierung]
%     → Regler DEAKTIVIERT, Integrator GESPERRT
%     → Beobachter AKTIV (konvergiert bereits)
%
%   Phase 1 (t < t_precharge + t_capture):
%     → I_cmd = I0_vec                         [Ströme konstant]
%     → Regler AKTIVIERT mit r = y_sens        [Fehler ≈ 0 beim Einrasten]
%     → Integrator INITIALISIERT: z = 0, x_hat = Messung
%
%   Phase 2 (t < t_precharge + t_capture + t_ramp):
%     → r(t) = r_quintic(t - t_precharge - t_capture)
%     → Regler AKTIV, Integrator AKTIV mit Anti-Windup
%
%   Phase 3 (t ≥ t_ss_total):
%     → r(t) = 0   (Nominallufspalt)
%     → Normalbetrieb
%--------------------------------------------------------------------------

% Soft-Start-Parameterstruktur
ss_params.t_precharge    = t_precharge;      % [s] Dauer Phase 0
ss_params.t_capture      = t_capture;        % [s] Dauer Phase 1
ss_params.t_ramp         = t_ramp;           % [s] Dauer Phase 2
ss_params.t_settle       = t_settle;         % [s] Einschwingzeit Phase 3
ss_params.t_ss_total     = t_total;          % [s] Gesamtdauer Soft-Start

ss_params.v_ramp         = v_ramp;           % [m/s] Gewählte Rampenrate
ss_params.v_ramp_max     = v_ramp_max;       % [m/s] Maximale sichere Rampenrate
ss_params.delta_x_safe   = delta_x_safe;     % [m]   Sicherer Abweichungsbereich
ss_params.delta_init_max = delta_init_max;   % [m]   Auslegungsdevianz

% Quintic-Polynomkoeffizienten für Simulink-Lookup oder MATLAB Function Block
% r(τ) = δ_init · (c0 + c3·τ³ + c4·τ⁴ + c5·τ⁵)
% [c0, c1, c2, c3, c4, c5] = [1, 0, 0, -10, 15, -6]
ss_params.quintic_coeffs = [1, 0, 0, -10, 15, -6];   % Polynomkoeffizienten

% Diskrete Lookup-Tabelle der normierten Trajektorie (für Simulink 1-D Lookup)
N_lut = 1000;
tau_lut = linspace(0, 1, N_lut)';
ss_params.lut_tau       = tau_lut;                             % Normierte Zeit [0,1]
ss_params.lut_r_norm    = quintic_h(tau_lut);                  % Normierte Position [0,1]
ss_params.lut_dr_norm   = quintic_dh(tau_lut) * t_ramp;        % Normierte Geschwindigkeit

if ~exist('Parameters', 'dir'), mkdir('Parameters'); end
save(".\Parameters\SoftStartParams", "ss_params");

fprintf('\n=== SS6: Export ===\n');
fprintf('  Soft-Start-Parameter gespeichert: Parameters/SoftStartParams.mat\n');

%% Zusammenfassung
fprintf('\n');
fprintf('=================================================================================\n');
fprintf('[INFO] Soft-Start-Entwurf abgeschlossen.\n');
fprintf('[INFO] Phasendauern:\n');
fprintf('         Phase 0 Vormagnetisierung: %5.0f ms\n', t_precharge*1e3);
fprintf('         Phase 1 Einrasten:         %5.0f ms\n', t_capture*1e3);
fprintf('         Phase 2 Rampe:             %5.0f ms\n', t_ramp*1e3);
fprintf('         Phase 3 Einschwingen:      %5.0f ms\n', t_settle*1e3);
fprintf('         Gesamt:                    %5.0f ms\n', t_total*1e3);
fprintf('[INFO] Sichere Rampenrate: %.2f mm/s (max %.2f mm/s)\n', ...
        v_ramp*1e3, v_ramp_max*1e3);
fprintf('=================================================================================\n');
