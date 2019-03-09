clear();
clf();

f = scf(0);
clf(f, "clear");
f.figure_size = [1500, 1000];

cd "/home/yusaku/md201x/logs";

filename = "20190308_22-39-16";

raw = csvRead("./" + filename + ".csv");

raw_bounds = 2:5000;

t = raw(raw_bounds, 1);

t = t - t(2)

t_bounds = [0; 9000];

subplot(3, 1, 1);

vel_cur = raw(raw_bounds, 3);
vel_tgt = raw(raw_bounds, 4);

vel_cur_filtered = zeros(size(vel_cur));

w = 0.9;
vel_cur_filtered(1) = vel_cur(1);

for n = 1: size(vel_cur, 'r') - 1 do
    vel_cur_filtered(n + 1) = (w * vel_cur_filtered(n)) + ((1.0 - w) * vel_cur(n));
end

plot(t, vel_tgt, "b-");
plot(t, vel_cur, "r-");
plot(t, vel_cur_filtered, "g-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
legend(["$\omega_{target}$", "$\omega_{current}$", "$\omega_{filtered}$"], 3);
xgrid();

subplot(3, 1, 2);

u_p = raw(raw_bounds, 7);
u_i = raw(raw_bounds, 8);

plot(t, u_p, "b-");
plot(t, u_i, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
a.data_bounds(:, 2) = [-5; 5];
legend(["$\Delta u_p$", "$\Delta u_i$"], 3);
xgrid();

subplot(3, 1, 3);

tgt_trq = raw(raw_bounds, 9);
tgt_vol = raw(raw_bounds, 10);

plot(t, tgt_trq, "b-");
plot(t, tgt_vol, "r-");

a = gca();
a.tight_limits = "on";
a.data_bounds(:, 1) = t_bounds;
legend(["$\tau_{target}$", "$V_{target}$"], 3);
xgrid();

xs2pdf(0, "./figs/md201x_" + filename + ".pdf");
xs2png(0, "./figs/md201x_" + filename + ".png");



