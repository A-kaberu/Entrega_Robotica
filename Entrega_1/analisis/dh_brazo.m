%% Brazo robotico 4 GDL - Denavit-Hartenberg (Lab. 1 Fundamentos de Robotica, USACH 2026)
% Mismos parametros que firmware/src/config.h y analisis/dh_brazo.py.
% Ejecutar por secciones (Ctrl+Enter). La seccion simbolica necesita Symbolic Math Toolbox.

clear; clc;
L1 = 32; L2 = 67; L3 = 60.4; L4 = 85;   % [mm]
a1 = -1.2;                              % desplazamiento horizontal del hombro [mm]
H_BASE = 35;                            % mesa -> sistema {0} [mm]

% Tabla D-H estandar: [theta d a alpha]
tablaDH = @(q) [q(1) L1 a1 90;
                q(2) 0  L2  0;
                q(3) 0  L3  0;
                q(4) 0  L4  0];

% Matriz homogenea D-H (angulos en grados)
A = @(th, d, a, al) [cosd(th) -sind(th)*cosd(al)  sind(th)*sind(al) a*cosd(th);
                     sind(th)  cosd(th)*cosd(al) -cosd(th)*sind(al) a*sind(th);
                     0         sind(al)           cosd(al)          d;
                     0         0                  0                 1];

%% 1) Cinematica directa en HOME
qHome = [0 90 -90 0];
[T04, O] = fkBrazo(qHome, tablaDH, A, H_BASE);
disp('T0_4 en HOME ='); disp(T04);
disp('Origenes O0..O4 sobre la mesa [mm] ='); disp(O);

%% 2) Forma simbolica (opcional)
if exist('syms', 'file')
    syms q1 q2 q3 q4 real
    Asym = @(th, d, a, al) [cos(th) -sin(th)*cos(al)  sin(th)*sin(al) a*cos(th);
                            sin(th)  cos(th)*cos(al) -cos(th)*sin(al) a*sin(th);
                            0        sin(al)          cos(al)         d;
                            0        0                0               1];
    T = Asym(q1, L1, a1, sym(pi)/2) * Asym(q2, 0, L2, 0) * Asym(q3, 0, L3, 0) * Asym(q4, 0, L4, 0);
    T = simplify(T);
    disp('Posicion de la punta (simbolica, sistema {0}):'); disp(T(1:3, 4));
end

%% 3) Cinematica inversa y comprobacion
p = [63.6 63.6 5];      % punto de recogida 3, punta 5 mm sobre la mesa
phi = -90;              % pinza vertical hacia abajo
q = ikBrazo(p, phi, L1, L2, L3, L4, a1, H_BASE);
[~, Oq] = fkBrazo(q, tablaDH, A, H_BASE);
fprintf('q = [%.2f %.2f %.2f %.2f] deg, error FK = %.2e mm\n', q, norm(Oq(5, :) - p));

%% 4) Dibujo del brazo en HOME y en el punto anterior
figure; hold on; grid on; axis equal; view(35, 25);
xlabel('x [mm]'); ylabel('y [mm]'); zlabel('z [mm]');
for qq = {qHome, q}
    [~, Oi] = fkBrazo(qq{1}, tablaDH, A, H_BASE);
    plot3([0; Oi(:, 1)], [0; Oi(:, 2)], [0; Oi(:, 3)], '-o', 'LineWidth', 3);
end
legend('HOME', 'agarre punto 3'); title('Brazo 4 GDL - cinematica D-H');

%% Funciones
function [T, O] = fkBrazo(q, tablaDH, A, H_BASE)
    tab = tablaDH(q);
    T = eye(4);
    O = [0 0 H_BASE];
    for i = 1:4
        T = T * A(tab(i, 1), tab(i, 2), tab(i, 3), tab(i, 4));
        O(end + 1, :) = T(1:3, 4)' + [0 0 H_BASE]; %#ok<AGROW>
    end
end

function q = ikBrazo(p, phi, L1, L2, L3, L4, a1, H_BASE)
    % Desacople de muneca, solucion codo arriba (igual que el firmware)
    r  = hypot(p(1), p(2));
    rw = r - a1 - L4 * cosd(phi);
    zw = (p(3) - H_BASE) - L1 - L4 * sind(phi);
    D  = (rw^2 + zw^2 - L2^2 - L3^2) / (2 * L2 * L3);
    assert(abs(D) <= 1, 'Punto fuera de alcance');
    q3 = atan2d(-sqrt(1 - D^2), D);
    q2 = atan2d(zw, rw) - atan2d(L3 * sind(q3), L2 + L3 * cosd(q3));
    q = [atan2d(p(2), p(1)) q2 q3 phi - q2 - q3];
end
