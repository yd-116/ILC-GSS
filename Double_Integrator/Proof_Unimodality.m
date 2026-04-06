%% ========================= Parameters =========================
Ts    = 0.01;
N     = 100;
A     = [1, Ts; 0, 1];
B     = [Ts^2/2; Ts];
C     = [1, 0];
t     = (0:N-1) * Ts;
y_ref = sin(2 * pi * 1 * t);

%% ===================== Markov Parameters ======================
h = zeros(N, 1);
for i = 1:N
    if i == 1
        h(i) = C * B;
    else
        h(i) = C * A^(i-1) * B;
    end
end

%% ====================== Lifted Matrix ========================
G       = toeplitz(h, [h(1); zeros(N-1, 1)]);
P       = G * G';
eta_max = 2 / max(eig(P));

%% ================== Compute Performance Index =================
k_list    = [5, 10, 15, 20];
eta_range = linspace(0.1, 24, 100);
J_mat     = zeros(4, 100);

for k_idx = 1:4
    k = k_list(k_idx);
    for eta_idx = 1:100
        eta = eta_range(eta_idx);
        u   = zeros(N, 1);
        
        for i = 1:k
            e = y_ref - G * u;
            u = u + eta * G' * e;
        end
        
        e_k                      = y_ref - G * u;
        J_mat(k_idx, eta_idx)    = norm(e_k, 2);
    end
end

%% =========================== Plot =============================
figure('Color','white','Position', [1000, 100, 1300, 1000]);

legend_labels = {'RSS Error','Optimal Gain','Stability Boundary'};
color_blue    = [0, 0.4470, 0.7410];
color_red     = [0.85, 0.10, 0.10];

for k_idx = 1:4
    subplot(2, 2, k_idx); 
    hold on; 
    box on;
    
    plot(eta_range, J_mat(k_idx, :), ...
         'Color', color_blue, 'LineWidth', 3);
    
    [minJ, idx] = min(J_mat(k_idx, :));
    plot(eta_range(idx), minJ, ...
         'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', color_red, ...
         'MarkerEdgeColor', color_red);
    
    plot([eta_max, eta_max], ylim, ...
         'Color', color_red, ...
         'LineStyle', '--', ...
         'LineWidth', 3);
    
    title(['k = ', num2str(k_list(k_idx))], ...
          'FontSize', 26, 'interpreter','latex');
    xlabel('Learning Gain', ...
           'FontSize', 24, 'interpreter','latex');
    xlim([-1, 26]);
    
    ylabel('$\|\mathbf{e}\|_2$ (rad)', ...
           'FontSize', 24, 'interpreter','latex'); 
    set(gca, 'FontSize', 24, 'LineWidth', 1, 'TickDir','out');
end

set(gcf, 'Units', 'normalized');
hSubs = flip(findobj(gcf, 'Type', 'axes'));

hSubs(1).Position = [0.15, 0.60, 0.30, 0.30];
hSubs(2).Position = [0.55, 0.60, 0.30, 0.30];
hSubs(3).Position = [0.15, 0.15, 0.30, 0.30];
hSubs(4).Position = [0.55, 0.15, 0.30, 0.30];

hl = legend(legend_labels);
set(hl, 'Position', [0.28, 0, 0.45, 0.06], ...
       'Orientation', 'horizontal', ...
       'Box', 'off', ...
       'FontSize', 22, ...
       'interpreter','latex');