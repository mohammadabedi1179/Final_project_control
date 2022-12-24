R = 8.3;
L = 2.03 * 10^(-3);
N = 193;
nou = 0.836;
K_w = 93.1;
K_t = 0.0107;
J_m = 8.68 * 10^(-8);
b_m = 8.87 * 10^(-8);
K_p = 320;
K_d = 5;
K_i = 20.48;
J_l = 0;
s = tf('s');

F = tf([N*nou*K_t*K_d, N*nou*K_t*K_p, N*nou*K_t*K_i], [L*(J_l + J_m*N^2*nou), R*(J_l+J_m*N^2*nou)+L*b_m*N^2*nou, b_m*N^2*nou*R+ K_t*N^2*nou/K_w + N*nou*K_t*K_d, N*nou*K_t*K_p, N*nou*K_t*K_i]);
G = tf(6.9,[0.00000548, 0.02244, 3.602, 0]);
H = 1/(1/G + 1);
step(F)