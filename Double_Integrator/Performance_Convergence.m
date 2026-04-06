clear; clc; close all;

%% ========================= Parameters =========================
Ts = 0.01;
N  = 100;
A  = [1, Ts; 0, 1];
B  = [Ts^2/2; Ts];
C  = [1, 0];
t  = (0:N-1)*Ts;
y_ref = sin(2*pi*1*t);

%% ===================== Markov Parameters =====================
h = zeros(N, 1);
for i = 1:N
    if i == 1
        h(i) = C * B;
    else
        h(i) = C * A^(i-1) * B;
    end
end

%% ====================== Lifted Matrix =======================
G = toeplitz(h, [h(1); zeros(N-1, 1)]);
P = G * G';
eta_max = 2 / max(eig(P));

%% ===================== Cost Function ========================
function J = cost_eta(eta, G, y_ref, k, N)
    u = zeros(N, 1);
    for i = 1:k
        e = y_ref - G * u;
        u = u + eta * G' * e;
    end
    e_final = y_ref - G * u;
    J = norm(e_final);
end

%% ================== Golden-Section Search ====================
function eta_opt = golden_section_search(G, y_ref, k_target, N)
    a = 0.1;
    b = 24;
    max_iter = 10;
    gr = (sqrt(5)-1) / 2;
    
    c = b - gr * (b - a);
    d = a + gr * (b - a);
    Jc = cost_eta(c, G, y_ref, k_target, N);
    Jd = cost_eta(d, G, y_ref, k_target, N);
    
    for iter = 1:max_iter
        if Jc < Jd
            b = d;
            d = c;
            Jd = Jc;
            c = b - gr * (b - a);
            Jc = cost_eta(c, G, y_ref, k_target, N);
        else
            a = c;
            c = d;
            Jc = Jd;
            d = a + gr * (b - a);
            Jd = cost_eta(d, G, y_ref, k_target, N);
        end
    end
    eta_opt = (a + b) / 2;
end

%% ===================== ILC Simulation ========================
function err_hist = ilc_run(G, y_ref, eta, k_max, N)
    u = zeros(N, 1);
    err_hist = zeros(1, k_max+1);
    e = y_ref - G * u;
    err_hist(1) = norm(e);
    
    for k = 1:k_max
        u = u + eta * G' * e;
        e = y_ref - G * u;
        err_hist(k+1) = norm(e);
    end
end

%% ========================= Plot ==============================
k_list = [5, 10, 15, 20];
f = figure('Color','white','Position',[1000, 100, 1300, 1000]);

color_blue  = [0, 0.4470, 0.7410];
color_red   = [0.85, 0.10, 0.10];
color_green = [0.20, 0.80, 0.20];

for k_idx = 1:4
    k = k_list(k_idx);
    eta_opt = golden_section_search(G, y_ref, k, N);
    
    it = 0:k;
    e_opt = ilc_run(G, y_ref, eta_opt, k, N);
    e_con = ilc_run(G, y_ref, 1.0, k, N);
    e_agg = ilc_run(G, y_ref, 24.0, k, N);
    
    subplot(2, 2, k_idx); hold on; box on;
    plot(it, e_opt, '-', 'LineWidth', 3, 'Color', color_blue);
    plot(it, e_con, '--', 'LineWidth', 3, 'Color', color_red);
    plot(it, e_agg, '-.', 'LineWidth', 3, 'Color', color_green);
    
    title(['Max Iteration ', num2str(k)], 'FontSize', 26, 'interpreter','latex');
    xlabel('Iteration $k$', 'FontSize', 24, 'interpreter','latex');
    xlim([0, k]);
    ylabel('$\|\mathbf{e}_k\|_2$ (rad)', 'FontSize', 24, 'interpreter','latex');
    set(gca, 'FontSize', 24, 'LineWidth', 1, 'TickDir','out');
end

set(f, 'Units','normalized');
hSubs = flip(findobj(f, 'Type','axes'));
hSubs(1).Position = [0.15, 0.60, 0.30, 0.30];
hSubs(2).Position = [0.55, 0.60, 0.30, 0.30];
hSubs(3).Position = [0.15, 0.15, 0.30, 0.30];
hSubs(4).Position = [0.55, 0.15, 0.30, 0.30];

hl = legend('GSS-Optimized','Conservative $\eta=1$','Aggressive $\eta=24$');
set(hl, 'Position',[0.28, 0.0, 0.45, 0.06], ...
       'Orientation','horizontal', 'Box','off', ...
       'FontSize',22, 'interpreter','latex');